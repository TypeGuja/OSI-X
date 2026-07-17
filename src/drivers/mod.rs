//! Аппаратные драйверы прошивки.
//!
//! Содержит [`motor`] (двигатели осей). Конкретные драйверы
//! (`Tmc2209Driver` для X/Y, `Uln2003Driver` для Z) собираются из пинов
//! `Board` в `hardware_build::build_axes` и используются `App` через
//! `motion::StepGenerator`.

pub mod motor;
