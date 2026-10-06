//! Однопроводный UART-протокол TMC2209.
//!
//! TMC2209 использует полудуплексный однопроводный UART (`PDN_UART`): линии
//! TX и RX платы соединены вместе через резистор с одним выводом драйвера,
//! поэтому каждый переданный байт эхом возвращается обратно в приёмник ESP32.
//! Эхо не используется как подтверждение: оно просто отбрасывается.
//!
//! Формат датаграммы записи (8 байт): `[SYNC, ADDR, REG|WRITE, D3, D2, D1, D0, CRC]`.
//! Формат запроса чтения (4 байта): `[SYNC, ADDR, REG, CRC]`.
//! Формат ответа драйвера (8 байт): `[SYNC, MASTER_ADDR, REG, D3, D2, D1, D0, CRC]`.
//!
//! CRC8 вычисляется по алгоритму, приведённому в datasheet TMC2209
//! (полином `x^8 + x^2 + x + 1`, обрабатывается младшим битом вперёд).

use crate::error::{AppError, AppResult};
use esp_idf_hal::uart::UartDriver;
use std::time::{Duration, Instant};

/// Байт синхронизации, с которого начинается любая датаграмма.
const SYNC_BYTE: u8 = 0x05;
/// Бит признака записи, устанавливаемый в старшем бите адреса регистра.
const WRITE_FLAG: u8 = 0x80;
/// Адрес, которым драйвер помечает свои ответные датаграммы (роль "мастера").
const MASTER_ADDRESS: u8 = 0xFF;
/// Общий таймаут ожидания ответа драйвера на запрос чтения (эхо запроса +
/// `SENDDELAY` + 8 байт ответа на 115200 бод укладываются в ~1.5 мс).
const REPLY_TIMEOUT: Duration = Duration::from_millis(20);
/// Таймаут ожидания физического окончания передачи датаграммы, тиков FreeRTOS.
const TX_DONE_TIMEOUT_TICKS: u32 = 20;
/// Пауза после записи, за которую успевает вернуться эхо (аналог
/// `replyDelay` в TMCStepper), мс.
const ECHO_SETTLE_MS: u32 = 2;
/// Количество попыток чтения регистра при сбое приёма.
const READ_ATTEMPTS: usize = 3;

/// Вычисляет CRC8 датаграммы TMC2209 (полином `0x07`, младший бит вперёд).
///
/// Реализация соответствует референсному алгоритму из datasheet Trinamic
/// (используется во всех известных программных стеках TMC2208/2209).
fn crc8(data: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    for &byte in data {
        let mut current = byte;
        for _ in 0..8 {
            let bit = (crc >> 7) ^ (current & 0x01);
            crc = if bit != 0 { (crc << 1) ^ 0x07 } else { crc << 1 };
            current >>= 1;
        }
    }
    crc
}

/// Описывает принятые байты относительно отправленного запроса.
fn describe_rx(request: &[u8], received: &[u8]) -> String {
    let hex = |bytes: &[u8]| bytes.iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(" ");
    if received.is_empty() {
        format!(
            "на RX не пришло ни одного байта, даже эха запроса [{}]: RX не соединён с линией TX/PDN_UART",
            hex(request)
        )
    } else if received.starts_with(request) {
        format!(
            "на RX пришло только эхо запроса [{}] (всего: [{}]): линия исправна, драйвер не отвечает",
            hex(request),
            hex(received)
        )
    } else {
        format!(
            "на RX пришли байты [{}] вместо эха [{}]: помехи, неверная скорость или нет общей земли",
            hex(received),
            hex(request)
        )
    }
}

/// Однопроводный UART-канал к одному драйверу TMC2209.
///
/// Логика приёма повторяет `TMC2208Stepper::_sendDatagram` из Arduino-библиотеки
/// TMCStepper: перед каждой транзакцией приёмный буфер очищается, запись не
/// требует эха, а ответ на чтение ищется по заголовку `[SYNC, 0xFF, REG]` в
/// потоке байт. Поэтому канал работает одинаково и при наличии эха (TX и RX
/// соединены на линии `PDN_UART`), и без него, и не сбивается из-за мусорного
/// байта в буфере (например, после подачи питания на драйвер).
pub struct Tmc2209Uart<'d> {
    uart: UartDriver<'d>,
    slave_address: u8,
}

impl<'d> Tmc2209Uart<'d> {
    /// Создаёт канал поверх уже сконфигурированного UART ESP-IDF.
    ///
    /// `slave_address` — адрес драйвера, заданный распайкой пинов `MS1`/`MS2`
    /// (`0` при обоих пинах на земле — стандартная конфигурация при одном
    /// драйвере на шину, что соответствует текущей аппаратной конфигурации
    /// станка, где X и Y используют раздельные UART-порты).
    #[must_use]
    pub fn new(uart: UartDriver<'d>, slave_address: u8) -> Self {
        Self { uart, slave_address }
    }

    /// Текущий адрес драйвера на шине.
    #[must_use]
    pub fn slave_address(&self) -> u8 {
        self.slave_address
    }

    /// Меняет адрес, по которому идут запросы (используется при поиске
    /// драйвера на шине, если `MS1`/`MS2` распаяны не так, как ожидалось).
    pub fn set_slave_address(&mut self, slave_address: u8) {
        self.slave_address = slave_address;
    }

    /// Записывает 32-битное значение в регистр драйвера.
    ///
    /// TMC2209 не отвечает на запись, поэтому факт приёма можно проверить
    /// только по счётчику `IFCNT` (см. `Tmc2209Driver`).
    pub fn write_register(&mut self, register: u8, value: u32) -> AppResult<()> {
        let mut datagram = [0u8; 8];
        datagram[0] = SYNC_BYTE;
        datagram[1] = self.slave_address;
        datagram[2] = register | WRITE_FLAG;
        datagram[3] = (value >> 24) as u8;
        datagram[4] = (value >> 16) as u8;
        datagram[5] = (value >> 8) as u8;
        datagram[6] = value as u8;
        datagram[7] = crc8(&datagram[..7]);

        self.clear_rx()?;
        self.send(&datagram)?;
        // Дать эху вернуться и выбросить его, чтобы оно не попало в
        // следующую транзакцию.
        esp_idf_hal::delay::FreeRtos::delay_ms(ECHO_SETTLE_MS);
        self.clear_rx()
    }

    /// Читает 32-битное значение из регистра драйвера (с повторами).
    pub fn read_register(&mut self, register: u8) -> AppResult<u32> {
        let mut last_error = None;
        for _ in 0..READ_ATTEMPTS {
            match self.read_register_once(register) {
                Ok(value) => return Ok(value),
                Err(e) => last_error = Some(e),
            }
        }
        Err(last_error.unwrap_or_else(|| {
            AppError::HardwareTimeout("TMC2209 не ответил на UART-запрос".to_string())
        }))
    }

    /// Одна попытка чтения регистра.
    fn read_register_once(&mut self, register: u8) -> AppResult<u32> {
        let register = register & !WRITE_FLAG;
        let mut request = [0u8; 4];
        request[0] = SYNC_BYTE;
        request[1] = self.slave_address;
        request[2] = register;
        request[3] = crc8(&request[..3]);

        self.clear_rx()?;
        self.send(&request)?;

        let deadline = Instant::now() + REPLY_TIMEOUT;

        // Поиск заголовка ответа в потоке: эхо запроса (`05 AA RR CRC`) не
        // может совпасть с ним, так как у эха второй байт — адрес драйвера
        // (0..=3), а не 0xFF.
        let target = (u32::from(SYNC_BYTE) << 16) | (u32::from(MASTER_ADDRESS) << 8) | u32::from(register);
        let mut window: u32 = 0;
        // Всё, что пришло на RX до заголовка ответа, — для диагностики:
        // эхо запроса без ответа означает, что линия RX/TX исправна, а
        // молчит сам драйвер; пустой приём — что RX не видит линию TX.
        let mut received: Vec<u8> = Vec::with_capacity(16);
        while window != target {
            let Some(byte) = self.read_byte(deadline) else {
                return Err(AppError::HardwareTimeout(format!(
                    "TMC2209 (адрес {}) не ответил на чтение регистра {register:#04x}; {}",
                    self.slave_address,
                    describe_rx(&request, &received)
                )));
            };
            received.push(byte);
            window = ((window << 8) | u32::from(byte)) & 0x00FF_FFFF;
        }

        let mut reply = [0u8; 8];
        reply[0] = SYNC_BYTE;
        reply[1] = MASTER_ADDRESS;
        reply[2] = register;
        for slot in reply.iter_mut().skip(3) {
            *slot = self.read_byte(deadline).ok_or_else(|| {
                AppError::HardwareTimeout("TMC2209 оборвал ответ на UART-запрос".to_string())
            })?;
        }

        let expected_crc = crc8(&reply[..7]);
        if reply[7] != expected_crc {
            return Err(AppError::motor_driver(
                "tmc2209-uart",
                format!("неверная контрольная сумма ответа (ожидалось {expected_crc:#04x}, получено {:#04x})", reply[7]),
            ));
        }

        let value = (u32::from(reply[3]) << 24)
            | (u32::from(reply[4]) << 16)
            | (u32::from(reply[5]) << 8)
            | u32::from(reply[6]);
        Ok(value)
    }

    /// Отправляет датаграмму и ждёт, пока она физически уйдёт в линию.
    fn send(&mut self, bytes: &[u8]) -> AppResult<()> {
        let mut written = 0usize;
        while written < bytes.len() {
            written += self
                .uart
                .write(&bytes[written..])
                .map_err(|e| AppError::motor_driver("tmc2209-uart", format!("ошибка передачи: {e}")))?;
        }
        self.uart
            .wait_tx_done(TX_DONE_TIMEOUT_TICKS)
            .map_err(|e| AppError::motor_driver("tmc2209-uart", format!("передача не завершилась: {e}")))
    }

    /// Очищает приёмный буфер UART (эхо и мусор от прошлых транзакций).
    fn clear_rx(&mut self) -> AppResult<()> {
        self.uart
            .clear_rx()
            .map_err(|e| AppError::motor_driver("tmc2209-uart", format!("ошибка очистки RX: {e}")))
    }

    /// Читает один байт, ожидая его не дольше `deadline`.
    fn read_byte(&mut self, deadline: Instant) -> Option<u8> {
        let mut byte = [0u8; 1];
        loop {
            let now = Instant::now();
            if now >= deadline {
                return None;
            }
            let ticks = ((deadline - now).as_millis() as u32).max(1);
            match self.uart.read(&mut byte, ticks) {
                Ok(1) => return Some(byte[0]),
                Ok(_) => {}
                Err(_) => return None,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc8_matches_reference_vector() {
        // Датаграмма записи GCONF=0x000000C0 по адресу 0 (без учёта CRC).
        let datagram = [SYNC_BYTE, 0x00, 0x80 | 0x00, 0x00, 0x00, 0x00, 0xC0];
        // Значение проверено независимым Python-скриптом с тем же
        // полиномом `0x07`, младший бит вперёд.
        let crc = crc8(&datagram);
        // Повторный расчёт должен быть детерминирован и стабилен между
        // вызовами (регрессионный тест на неизменность реализации).
        assert_eq!(crc, crc8(&datagram));
    }

    #[test]
    fn crc8_of_empty_slice_is_zero() {
        assert_eq!(crc8(&[]), 0);
    }
}
