//! Драйвер статусного адресного светодиода WS2812 на периферии RMT.
//!
//! Используется для визуальной индикации состояния станка (инициализация,
//! готовность, печать, ошибка) без необходимости смотреть в лог/веб-интерфейс.

use crate::error::{AppError, AppResult};
use esp_idf_hal::rmt::{
    config::TransmitConfig, FixedLengthSignal, PinState, Pulse, PulseTicks, RmtChannel,
    TxRmtDriver,
};

/// Длительности импульсов WS2812 в наносекундах (спецификация чипа).
mod timing {
    pub const T0H_NS: u64 = 350;
    pub const T0L_NS: u64 = 900;
    pub const T1H_NS: u64 = 900;
    pub const T1L_NS: u64 = 350;
    /// Минимальная пауза после кадра данных, требуемая чипом для
    /// защёлкивания значения (документирует требование протокола;
    /// линия и так остаётся в `Low` между вызовами `set_color`).
    #[allow(dead_code)]
    pub const RESET_NS: u64 = 280_000;
}

/// Цвет в формате RGB (без гамма-коррекции и без альфа-канала).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Color {
    /// Красный канал.
    pub r: u8,
    /// Зелёный канал.
    pub g: u8,
    /// Синий канал.
    pub b: u8,
}

impl Color {
    /// Чёрный (светодиод выключен).
    pub const OFF: Color = Color { r: 0, g: 0, b: 0 };
    /// Индикация нормальной готовности станка.
    pub const READY: Color = Color { r: 0, g: 40, b: 0 };
    /// Индикация процесса печати.
    pub const PRINTING: Color = Color { r: 0, g: 0, b: 40 };
    /// Индикация предупреждения.
    pub const WARNING: Color = Color { r: 40, g: 40, b: 0 };
    /// Индикация критической ошибки.
    pub const ERROR: Color = Color { r: 60, g: 0, b: 0 };
    /// Индикация инициализации при старте.
    pub const BOOT: Color = Color { r: 0, g: 0, b: 15 };
}

/// Статусный светодиод WS2812, управляемый через один канал RMT.
pub struct RgbStatus<'d> {
    driver: TxRmtDriver<'d>,
    t0h: Pulse,
    t0l: Pulse,
    t1h: Pulse,
    t1l: Pulse,
    current: Color,
}

impl<'d> RgbStatus<'d> {
    /// Создаёт драйвер статусного светодиода на указанном пине и канале RMT.
    pub fn new<C: RmtChannel>(
        pin: impl Into<esp_idf_hal::gpio::AnyOutputPin>,
        channel: impl esp_idf_hal::peripheral::Peripheral<P = C> + 'd,
    ) -> AppResult<Self> {
        let config = TransmitConfig::new().clock_divider(1);
        let driver = TxRmtDriver::new(channel, pin.into(), &config)
            .map_err(|e| AppError::board(format!("не удалось инициализировать RMT для RGB: {e}")))?;

        let ticks_hz = driver
            .counter_clock()
            .map_err(|e| AppError::board(format!("не удалось получить частоту RMT: {e}")))?;

        let ns_to_ticks = |ns: u64| -> AppResult<PulseTicks> {
            let ticks = (ticks_hz.0 as u64 * ns) / 1_000_000_000;
            PulseTicks::new(ticks as u16)
                .map_err(|e| AppError::board(format!("некорректная длительность импульса RMT: {e}")))
        };

        let t0h = Pulse::new(PinState::High, ns_to_ticks(timing::T0H_NS)?);
        let t0l = Pulse::new(PinState::Low, ns_to_ticks(timing::T0L_NS)?);
        let t1h = Pulse::new(PinState::High, ns_to_ticks(timing::T1H_NS)?);
        let t1l = Pulse::new(PinState::Low, ns_to_ticks(timing::T1L_NS)?);

        let mut status = Self {
            driver,
            t0h,
            t0l,
            t1h,
            t1l,
            current: Color::OFF,
        };
        status.set_color(Color::BOOT)?;
        Ok(status)
    }

    /// Устанавливает цвет светодиода немедленно.
    pub fn set_color(&mut self, color: Color) -> AppResult<()> {
        // Порядок байт WS2812 — GRB, старший бит первым.
        let bytes = [color.g, color.r, color.b];
        let mut signal = FixedLengthSignal::<24>::new();

        let mut index = 0usize;
        for byte in bytes {
            for bit_index in (0..8).rev() {
                let bit_is_set = (byte >> bit_index) & 0b1 != 0;
                let (high, low) = if bit_is_set {
                    (self.t1h, self.t1l)
                } else {
                    (self.t0h, self.t0l)
                };
                signal
                    .set(index, &(high, low))
                    .map_err(|e| AppError::board(format!("ошибка формирования сигнала RMT: {e}")))?;
                index += 1;
            }
        }

        self.driver
            .start_blocking(&signal)
            .map_err(|e| AppError::board(format!("не удалось передать сигнал WS2812: {e}")))?;

        self.current = color;
        Ok(())
    }

    /// Выключает светодиод.
    pub fn off(&mut self) -> AppResult<()> {
        self.set_color(Color::OFF)
    }

    /// Возвращает текущий установленный цвет.
    #[must_use]
    pub fn current_color(&self) -> Color {
        self.current
    }
}
