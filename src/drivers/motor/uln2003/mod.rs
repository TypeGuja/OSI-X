//! Драйвер двигателя на базе **ULN2003** (программное управление фазами).
//!
//! Используется осью Z на первом этапе (двигатель 28BYJ-48). Как и
//! `tmc2209`, модуль разделён на GPIO-обёртку ([`gpio`]) и собственно
//! драйвер ([`driver`]), реализующий
//! [`crate::drivers::motor::driver::MotorDriver`].

pub mod driver;
pub mod gpio;

pub use driver::{StepMode, Uln2003Driver};
pub use gpio::Uln2003Pins;
