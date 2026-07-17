//! Конфигурация принтера верхнего уровня: геометрия, кинематика, экструдер.

use serde::{Deserialize, Serialize};

/// Тип кинематической схемы станка.
///
/// На первом этапе поддерживается только [`KinematicsKind::Cartesian`];
/// остальные варианты зарезервированы для модуля `motion::kinematics`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum KinematicsKind {
    /// Декартова кинематика (независимые оси X/Y/Z).
    Cartesian,
    /// CoreXY.
    CoreXY,
    /// CoreXZ.
    CoreXZ,
    /// Дельта-кинематика.
    Delta,
}

impl Default for KinematicsKind {
    fn default() -> Self {
        Self::Cartesian
    }
}

/// Габариты рабочей зоны станка в миллиметрах.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BedSize {
    /// Размер по X.
    pub x_mm: f32,
    /// Размер по Y.
    pub y_mm: f32,
    /// Размер по Z (максимальная высота печати).
    pub z_mm: f32,
}

impl Default for BedSize {
    fn default() -> Self {
        Self {
            x_mm: 220.0,
            y_mm: 220.0,
            z_mm: 250.0,
        }
    }
}

/// Верхнеуровневая конфигурация принтера (`printer.toml`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct PrinterConfig {
    /// Человекочитаемое имя станка (отображается в веб-интерфейсе и логах).
    pub name: String,
    /// Тип кинематической схемы.
    pub kinematics: KinematicsKind,
    /// Размеры рабочей зоны.
    pub bed_size: BedSize,
    /// Признак наличия подогреваемого стола.
    pub has_heated_bed: bool,
    /// Количество экструдеров (на первом этапе только `1`).
    pub extruder_count: u8,
}

impl Default for PrinterConfig {
    fn default() -> Self {
        Self {
            name: "OSIX Printer".to_string(),
            kinematics: KinematicsKind::default(),
            bed_size: BedSize::default(),
            has_heated_bed: true,
            extruder_count: 1,
        }
    }
}
