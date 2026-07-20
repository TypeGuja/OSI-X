//! Драйвер двигателя на базе **TMC2209** (UART, STEP/DIR).
//!
//! Используется осями X и Y (NEMA17). Модуль разделён на:
//! - [`registers`] — адреса и типизированные регистры;
//! - [`uart`] — однопроводный UART-протокол (датаграммы, CRC8);
//! - [`status`] — разбор диагностического регистра `DRV_STATUS`;
//! - [`driver`] — собственно [`Tmc2209Driver`], реализующий
//!   [`crate::drivers::motor::driver::MotorDriver`].

pub mod driver;
pub mod registers;
pub mod status;
pub mod uart;

pub use driver::{CurrentSenseConfig, Tmc2209Driver};
pub use registers::{ChopConf, CoolConf, GConf, MicrostepResolution, PwmConf};
pub use status::DriverStatus;
pub use uart::Tmc2209Uart;
