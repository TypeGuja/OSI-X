//! Драйверы двигателей осей станка.
//!
//! [`driver::MotorDriver`] — единственная граница абстракции между
//! планировщиком движения и конкретным электрическим драйвером. На этом
//! этапе реализованы [`tmc2209::Tmc2209Driver`] (оси X, Y) и
//! [`uln2003::Uln2003Driver`] (ось Z, двигатель 28BYJ-48). Добавление
//! A4988, DRV8825, TMC2208 или TMC5160 сводится к новому подмодулю рядом с
//! `tmc2209`/`uln2003`, реализующему тот же трейт — без изменений в
//! `axis.rs`, `motion` или `gcode`.

pub mod axis;
pub mod driver;
pub mod tmc2209;
pub mod uln2003;

pub use axis::{Axis, AxisControl};
pub use driver::MotorDriver;
