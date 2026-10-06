//! Виртуальный последовательный порт поверх встроенного контроллера
//! USB-Serial-JTAG ESP32-S3 (нативный USB-разъём, на компьютере виден как
//! COM-порт / `ttyACM` без отдельного USB-UART моста).
//!
//! Раньше модуль использовал TinyUSB CDC-ACM, но `esp_tinyusb` — внешний
//! компонент ESP-IDF (менеджер компонентов), его функций нет в биндингах
//! `esp-idf-sys` по умолчанию, и прошивка не собиралась. Драйвер
//! USB-Serial-JTAG входит в сам ESP-IDF и даёт тот же функционал CDC.

use crate::error::{AppError, AppResult};
use esp_idf_sys::EspError;

/// Размер кольцевого буфера приёма драйвера, байт.
const RX_BUFFER_SIZE: u32 = 256;
/// Размер кольцевого буфера передачи драйвера, байт.
const TX_BUFFER_SIZE: u32 = 1024;
/// Максимальное ожидание места в буфере передачи, тиков FreeRTOS.
const WRITE_TIMEOUT_TICKS: u32 = 10;

/// USB CDC порт, готовый к побайтовому поллинговому чтению/записи.
///
/// Не реализует построчную сборку сама — этим занимается
/// [`crate::usb::SerialConsole`], работающий поверх трейта
/// [`crate::usb::SerialTransport`], которому эта структура и удовлетворяет.
pub struct UsbCdc;

impl UsbCdc {
    /// Устанавливает драйвер USB-Serial-JTAG.
    ///
    /// Должна вызываться ровно один раз за время жизни программы.
    pub fn install() -> AppResult<Self> {
        let mut config = esp_idf_sys::usb_serial_jtag_driver_config_t {
            tx_buffer_size: TX_BUFFER_SIZE,
            rx_buffer_size: RX_BUFFER_SIZE,
        };

        // SAFETY: `config` живёт на стеке до конца вызова; драйвер копирует
        // значения и не сохраняет указатель.
        let ret = unsafe { esp_idf_sys::usb_serial_jtag_driver_install(&mut config) };
        EspError::convert(ret)
            .map_err(|e| AppError::board(format!("не удалось установить драйвер USB-Serial-JTAG: {e}")))?;

        log::info!("USB CDC (USB-Serial-JTAG) инициализирован");
        Ok(Self)
    }

    /// Считывает доступные байты, не блокируясь. Возвращает `0`, если
    /// новых данных нет — это штатная ситуация при поллинге, а не ошибка.
    pub fn read(&mut self, buf: &mut [u8]) -> AppResult<usize> {
        // SAFETY: `buf` — валидный изменяемый срез на время вызова.
        let read = unsafe {
            esp_idf_sys::usb_serial_jtag_read_bytes(buf.as_mut_ptr().cast(), buf.len() as u32, 0)
        };
        if read < 0 {
            return Err(AppError::board("ошибка чтения USB CDC".to_string()));
        }
        Ok(read as usize)
    }

    /// Передаёт `bytes` (копирует в буфер передачи драйвера).
    pub fn write(&mut self, bytes: &[u8]) -> AppResult<()> {
        // SAFETY: `bytes` — валидный срез памяти на время вызова.
        let written = unsafe {
            esp_idf_sys::usb_serial_jtag_write_bytes(bytes.as_ptr().cast(), bytes.len(), WRITE_TIMEOUT_TICKS)
        };
        if written < 0 {
            return Err(AppError::board("не удалось передать данные по USB CDC".to_string()));
        }
        if written as usize != bytes.len() {
            log::warn!(
                "буфер передачи USB CDC переполнен: отправлено {written} из {} байт",
                bytes.len()
            );
        }
        Ok(())
    }
}
