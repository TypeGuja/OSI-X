//! Конфигурация подсистемы движения: параметры осей, ускорения, jerk,
//! junction deviation (`motion.toml`).

use crate::types::AxisId;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// Параметры одной оси, необходимые планировщику и генератору шагов.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct AxisMotionConfig {
    /// Количество шагов двигателя (с учётом микрошага) на один миллиметр
    /// перемещения оси.
    pub steps_per_mm: f32,
    /// Максимальная скорость оси, мм/с.
    pub max_speed_mm_s: f32,
    /// Максимальное ускорение оси, мм/с².
    pub max_acceleration_mm_s2: f32,
    /// Jerk (мгновенное изменение скорости без разгона), мм/с.
    pub jerk_mm_s: f32,
    /// Инвертировать направление вращения относительно положительного
    /// направления оси.
    pub invert_direction: bool,
    /// Минимальная координата оси (после хоуминга), мм.
    pub min_position_mm: f32,
    /// Максимальная координата оси, мм.
    pub max_position_mm: f32,
}

impl Default for AxisMotionConfig {
    fn default() -> Self {
        Self {
            steps_per_mm: 80.0,
            max_speed_mm_s: 200.0,
            max_acceleration_mm_s2: 1500.0,
            jerk_mm_s: 8.0,
            invert_direction: false,
            min_position_mm: 0.0,
            max_position_mm: 220.0,
        }
    }
}

/// Максимально допустимое отклонение траектории в узле стыка сегментов
/// (junction deviation), используемое планировщиком look-ahead для расчёта
/// безопасной скорости прохождения угла без разгона/торможения в ноль.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct JunctionDeviationConfig {
    /// Отклонение в миллиметрах.
    pub deviation_mm: f32,
}

impl Default for JunctionDeviationConfig {
    fn default() -> Self {
        Self {
            deviation_mm: 0.013,
        }
    }
}

/// Конфигурация подсистемы движения (`motion.toml`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct MotionConfig {
    /// Параметры каждой оси по идентификатору.
    pub axes: BTreeMap<String, AxisMotionConfig>,
    /// Настройки junction deviation для look-ahead планировщика.
    pub junction_deviation: JunctionDeviationConfig,
    /// Глубина очереди планировщика (количество одновременно
    /// просчитываемых look-ahead сегментов).
    pub planner_queue_depth: usize,
    /// Частота генератора шагов по умолчанию, Гц (верхняя граница,
    /// фактическая частота определяется скоростью и `steps_per_mm`).
    pub max_step_rate_hz: u32,
}

impl MotionConfig {
    /// Возвращает конфигурацию указанной оси, если она определена.
    #[must_use]
    pub fn axis(&self, axis: AxisId) -> Option<&AxisMotionConfig> {
        self.axes.get(axis.to_string().as_str())
    }
}

impl Default for MotionConfig {
    fn default() -> Self {
        let mut axes = BTreeMap::new();
        axes.insert(AxisId::X.to_string(), AxisMotionConfig::default());
        axes.insert(AxisId::Y.to_string(), AxisMotionConfig::default());
        axes.insert(
            AxisId::Z.to_string(),
            AxisMotionConfig {
                steps_per_mm: 4076.0 / 8.0, // 28BYJ-48 через ULN2003, полушаг, без редуктора винта
                max_speed_mm_s: 5.0,
                max_acceleration_mm_s2: 100.0,
                jerk_mm_s: 0.4,
                invert_direction: false,
                min_position_mm: 0.0,
                max_position_mm: 250.0,
            },
        );

        Self {
            axes,
            junction_deviation: JunctionDeviationConfig::default(),
            planner_queue_depth: 32,
            max_step_rate_hz: 200_000,
        }
    }
}
