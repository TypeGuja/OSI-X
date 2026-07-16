//! Модуль платы (board support): распиновка, питание, watchdog, индикация.
//!
//! `Board` — единственная точка входа для доступа к low-level ресурсам
//! платы. Остальные подсистемы прошивки (drivers, motion, gcode, ...)
//! получают уже сконфигурированные объекты (например, конкретный UART для
//! TMC2209) из `Board`, а не работают с `Peripherals` напрямую — это
//! позволяет держать распиновку в одном месте ([`pins::PinMap`]).
//!
//! `dead_code` отключён для полей `uart1`/`uart2`/`spi2`: они зарезервированы
//! под инициализацию `Tmc2209Uart` и SD-карты на следующих этапах и пока не
//! читаются — предупреждение будет снято, как только эти этапы подключат
//! соответствующую периферию.
#![allow(dead_code)]

pub mod pins;
pub mod power;
pub mod rgb;
pub mod watchdog;

use crate::error::AppResult;
use esp_idf_hal::adc::ADC1;
use esp_idf_hal::ledc::LEDC;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::spi::SPI2;
use esp_idf_hal::uart::{UART1, UART2};
use pins::PinMap;
use power::Power;
use rgb::RgbStatus;
use watchdog::Watchdog;

/// Таймаут Task Watchdog по умолчанию, секунд.
const DEFAULT_WATCHDOG_TIMEOUT_S: u32 = 5;

/// Плата OSIX: агрегирует питание, watchdog и индикацию.
///
/// Периферия, специфичная для конкретных драйверов (UART для TMC2209, SPI
/// для SD-карты, GPIO для STEP/DIR/ULN2003), забирается из [`Peripherals`]
/// на последующих этапах — на этапе 1 `Board` резервирует и настраивает
/// только общесистемные ресурсы, а карта пинов ([`PinMap`]) уже доступна
/// целиком для использования модулями `drivers` в следующих этапах.
pub struct Board<'d> {
    /// Управление силовым питанием и аварийной остановкой.
    pub power: Power<'d>,
    /// Task Watchdog Timer.
    pub watchdog: Watchdog,
    /// Статусный RGB-светодиод.
    pub rgb: RgbStatus<'d>,
    /// Карта пинов, из которой последующие модули (drivers, sdcard, ...)
    /// извлекают номера GPIO при собственной инициализации.
    pub pins: PinMap,
    /// Периферия UART1, зарезервированная под TMC2209 оси X (настраивается
    /// драйвером `drivers::motor::tmc2209` на следующем этапе).
    pub uart1: UART1,
    /// Периферия UART2, зарезервированная под TMC2209 оси Y.
    pub uart2: UART2,
    /// Периферия SPI2, зарезервированная под SD-карту.
    pub spi2: SPI2,
    /// Периферия LEDC (ШИМ), зарезервированная под нагреватели и вентилятор.
    pub ledc: LEDC,
    /// Периферия ADC1, зарезервированная под термисторы.
    pub adc1: ADC1,
}

impl<'d> Board<'d> {
    /// Инициализирует плату: захватывает периферию ESP-IDF, настраивает
    /// питание, watchdog и статусный светодиод.
    ///
    /// Периферия захватывается один раз за время жизни программы — повторный
    /// вызов [`Peripherals::take`] после первого успешного вызова паникует
    /// внутри `esp-idf-hal`, поэтому `Board::init` должна вызываться ровно
    /// один раз из `main`. Периферия, не используемая на этом этапе (UART1,
    /// UART2, SPI2), сохраняется в `Board` для последующего потребления
    /// модулями `drivers` и `storage`.
    pub fn init() -> AppResult<Self> {
        let peripherals = Peripherals::take()
            .map_err(|e| crate::error::AppError::board(format!("периферия уже захвачена: {e}")))?;

        let pins = PinMap::DEFAULT;

        let psu_pin = unsafe {
            esp_idf_hal::gpio::AnyOutputPin::new(pins.system.psu_enable as i32)
        };
        let estop_pin = unsafe {
            esp_idf_hal::gpio::AnyIOPin::new(pins.system.emergency_stop as i32)
        };
        let power = Power::new(psu_pin, estop_pin)?;

        let watchdog = Watchdog::init(DEFAULT_WATCHDOG_TIMEOUT_S)?;
        watchdog.add_current_task()?;

        let rgb_pin = unsafe { esp_idf_hal::gpio::AnyOutputPin::new(pins.system.status_rgb as i32) };
        let rgb = RgbStatus::new(rgb_pin, peripherals.rmt.channel0)?;

        log::info!("плата OSIX инициализирована (ESP32-S3 N16R8)");

        Ok(Self {
            power,
            watchdog,
            rgb,
            pins,
            uart1: peripherals.uart1,
            uart2: peripherals.uart2,
            spi2: peripherals.spi2,
            ledc: peripherals.ledc,
            adc1: peripherals.adc1,
        })
    }
}
