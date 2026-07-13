//! Распиновка платы OSIX на базе ESP32-S3 N16R8.
//!
//! Все номера GPIO собраны в одном месте, чтобы смена платы/распиновки не
//! требовала правок в логике драйверов — они принимают уже сконфигурированные
//! периферийные объекты, а не номера пинов напрямую.

/// Номер GPIO, представленный как `u8` (соответствует диапазону ESP32-S3:
/// 0..=48, из которых часть зарезервирована под flash/PSRAM и недоступна).
pub type GpioNum = u8;

/// Пины, относящиеся к оси X (NEMA17 + TMC2209, STEP/DIR + общий UART).
#[derive(Debug, Clone, Copy)]
pub struct AxisXPins {
    /// Пин STEP.
    pub step: GpioNum,
    /// Пин DIR.
    pub dir: GpioNum,
    /// Пин ENABLE (общий для драйвера, активен низким уровнем).
    pub enable: GpioNum,
    /// Пин концевого выключателя оси X.
    pub endstop: GpioNum,
}

/// Пины, относящиеся к оси Y (NEMA17 + TMC2209, STEP/DIR + общий UART).
#[derive(Debug, Clone, Copy)]
pub struct AxisYPins {
    /// Пин STEP.
    pub step: GpioNum,
    /// Пин DIR.
    pub dir: GpioNum,
    /// Пин ENABLE (активен низким уровнем).
    pub enable: GpioNum,
    /// Пин концевого выключателя оси Y.
    pub endstop: GpioNum,
}

/// Пины, относящиеся к оси Z (28BYJ-48 + ULN2003, 4 обмотки через GPIO).
///
/// При замене на TMC2209 в будущем эта структура заменяется на структуру
/// вида `AxisXPins`/`AxisYPins` — код, использующий [`crate::board::Board`],
/// не изменится, поскольку он работает через абстракцию `MotorDriver`,
/// а не через конкретные пины.
#[derive(Debug, Clone, Copy)]
pub struct AxisZPins {
    /// Обмотка A (IN1 ULN2003).
    pub in1: GpioNum,
    /// Обмотка B (IN2 ULN2003).
    pub in2: GpioNum,
    /// Обмотка C (IN3 ULN2003).
    pub in3: GpioNum,
    /// Обмотка D (IN4 ULN2003).
    pub in4: GpioNum,
    /// Пин концевого выключателя оси Z.
    pub endstop: GpioNum,
}

/// Пины общего UART-шлейфа TMC2209 (X и Y используют раздельные UART-порты
/// аппаратного ESP32-S3, чтобы избежать программного мультиплексирования
/// адресов на шине).
#[derive(Debug, Clone, Copy)]
pub struct TmcUartPins {
    /// TX для драйвера оси X (UART1).
    pub x_tx: GpioNum,
    /// RX для драйвера оси X (UART1).
    pub x_rx: GpioNum,
    /// TX для драйвера оси Y (UART2).
    pub y_tx: GpioNum,
    /// RX для драйвера оси Y (UART2).
    pub y_rx: GpioNum,
}

/// Пины подсистемы питания и индикации.
#[derive(Debug, Clone, Copy)]
pub struct SystemPins {
    /// Пин управления силовым реле/MOSFET блока питания (PS_ON).
    pub psu_enable: GpioNum,
    /// Пин адресной RGB-индикации статуса (WS2812, через RMT).
    pub status_rgb: GpioNum,
    /// Пин аварийной кнопки (E-Stop), активен низким уровнем.
    pub emergency_stop: GpioNum,
}

/// Пины подсистемы SD-карты (SPI).
#[derive(Debug, Clone, Copy)]
pub struct SdCardPins {
    /// SPI MOSI.
    pub mosi: GpioNum,
    /// SPI MISO.
    pub miso: GpioNum,
    /// SPI SCLK.
    pub sclk: GpioNum,
    /// SPI CS.
    pub cs: GpioNum,
    /// Пин детекта карты (Card Detect), опционален физически, но
    /// присутствует в распиновке для единообразия.
    pub card_detect: GpioNum,
}

/// Полная карта пинов платы OSIX (ESP32-S3 N16R8).
#[derive(Debug, Clone, Copy)]
pub struct PinMap {
    /// Пины оси X.
    pub axis_x: AxisXPins,
    /// Пины оси Y.
    pub axis_y: AxisYPins,
    /// Пины оси Z.
    pub axis_z: AxisZPins,
    /// Пины UART для TMC2209.
    pub tmc_uart: TmcUartPins,
    /// Системные пины (питание, индикация, E-Stop).
    pub system: SystemPins,
    /// Пины SD-карты.
    pub sd_card: SdCardPins,
}

impl PinMap {
    /// Распиновка по умолчанию для референсной платы OSIX rev.1.
    ///
    /// Значения выбраны так, чтобы не пересекаться со стандартными
    /// зарезервированными пинами ESP32-S3 (GPIO 26..=32 используются под
    /// PSRAM/Flash на модуле N16R8 и здесь не задействуются).
    pub const DEFAULT: PinMap = PinMap {
        axis_x: AxisXPins {
            step: 4,
            dir: 5,
            enable: 6,
            endstop: 7,
        },
        axis_y: AxisYPins {
            step: 15,
            dir: 16,
            enable: 17,
            endstop: 18,
        },
        axis_z: AxisZPins {
            in1: 8,
            in2: 9,
            in3: 10,
            in4: 11,
            endstop: 12,
        },
        tmc_uart: TmcUartPins {
            x_tx: 13,
            x_rx: 14,
            y_tx: 21,
            y_rx: 47,
        },
        system: SystemPins {
            psu_enable: 38,
            status_rgb: 48,
            emergency_stop: 1,
        },
        sd_card: SdCardPins {
            mosi: 35,
            miso: 37,
            sclk: 36,
            cs: 34,
            card_detect: 33,
        },
    };
}

impl Default for PinMap {
    fn default() -> Self {
        Self::DEFAULT
    }
}
