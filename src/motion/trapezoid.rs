//! Трапецеидальный профиль скорости для одного сегмента движения.
//!
//! Каждый сегмент, вышедший из [`crate::motion::planner::MotionPlanner`] с
//! уже вычисленными входной/выходной скоростью (см. `planner::recalculate`),
//! преобразуется в конкретный профиль разгона — участок разгона, участок
//! постоянной (крейсерской) скорости и участок торможения. Если сегмент
//! слишком короткий для выхода на крейсерскую скорость, разгон и торможение
//! сходятся в одной точке (треугольный профиль) на пониженной пиковой
//! скорости.

use crate::motion::acceleration::{distance_for_speed_change, velocity_after_distance};

/// Трапецеидальный (или вырожденный треугольный) профиль скорости одного
/// сегмента движения.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrapezoidProfile {
    /// Скорость в начале сегмента, мм/с.
    pub entry_speed_mm_s: f32,
    /// Пиковая (крейсерская) скорость сегмента, мм/с — может быть ниже
    /// запрошенной скорости подачи, если сегмент слишком короткий.
    pub cruise_speed_mm_s: f32,
    /// Скорость в конце сегмента, мм/с.
    pub exit_speed_mm_s: f32,
    /// Расстояние участка разгона, мм.
    pub accelerate_distance_mm: f32,
    /// Расстояние участка постоянной скорости, мм (может быть `0`).
    pub cruise_distance_mm: f32,
    /// Расстояние участка торможения, мм.
    pub decelerate_distance_mm: f32,
    /// Ускорение/замедление профиля, мм/с² (одно и то же значение для
    /// разгона и торможения — асимметричные профили не требуются для
    /// FDM-печати).
    pub acceleration_mm_s2: f32,
}

impl TrapezoidProfile {
    /// Полная длина сегмента (сумма трёх участков), мм.
    #[must_use]
    pub fn total_distance_mm(&self) -> f32 {
        self.accelerate_distance_mm + self.cruise_distance_mm + self.decelerate_distance_mm
    }

    /// Строит профиль для сегмента длиной `distance_mm` с заданными
    /// граничными скоростями и ограничениями.
    ///
    /// `entry_speed_mm_s` и `exit_speed_mm_s` должны быть уже согласованы
    /// планировщиком (не превышать `max_speed_mm_s` и физически достижимы
    /// при `acceleration_mm_s2` на данной дистанции) — эта функция строит
    /// профиль, но не проверяет корректность входных скоростей заново.
    #[must_use]
    pub fn build(
        distance_mm: f32,
        entry_speed_mm_s: f32,
        exit_speed_mm_s: f32,
        max_speed_mm_s: f32,
        acceleration_mm_s2: f32,
    ) -> Self {
        let entry = entry_speed_mm_s.max(0.0);
        let exit = exit_speed_mm_s.max(0.0);
        let max_speed = max_speed_mm_s.max(entry).max(exit);
        let distance = distance_mm.max(0.0);

        // Расстояние, необходимое для разгона от `entry` до `max_speed`, и
        // для торможения с `max_speed` до `exit`, при выходе на полную
        // крейсерскую скорость.
        let accel_to_cruise = distance_for_speed_change(entry, max_speed, acceleration_mm_s2).max(0.0);
        let decel_from_cruise = distance_for_speed_change(exit, max_speed, acceleration_mm_s2).max(0.0);

        if accel_to_cruise + decel_from_cruise <= distance {
            // Обычный трапецеидальный профиль: есть место для крейсерского участка.
            Self {
                entry_speed_mm_s: entry,
                cruise_speed_mm_s: max_speed,
                exit_speed_mm_s: exit,
                accelerate_distance_mm: accel_to_cruise,
                cruise_distance_mm: distance - accel_to_cruise - decel_from_cruise,
                decelerate_distance_mm: decel_from_cruise,
                acceleration_mm_s2,
            }
        } else {
            // Сегмент слишком короткий для полной крейсерской скорости —
            // вырожденный треугольный профиль с пониженной пиковой скоростью.
            // Пиковая скорость находится из равенства расстояний разгона и
            // торможения суммарной длине сегмента:
            //   (peak² - entry²)/(2a) + (peak² - exit²)/(2a) = distance
            //   peak² = a*distance + (entry² + exit²) / 2
            let peak_sq = acceleration_mm_s2 * distance + (entry * entry + exit * exit) / 2.0;
            let peak = peak_sq.max(0.0).sqrt().max(entry).max(exit);

            let accelerate_distance_mm =
                distance_for_speed_change(entry, peak, acceleration_mm_s2).max(0.0).min(distance);
            let decelerate_distance_mm = (distance - accelerate_distance_mm).max(0.0);

            Self {
                entry_speed_mm_s: entry,
                cruise_speed_mm_s: peak,
                exit_speed_mm_s: exit,
                accelerate_distance_mm,
                cruise_distance_mm: 0.0,
                decelerate_distance_mm,
                acceleration_mm_s2,
            }
        }
    }

    /// Скорость в точке, отстоящей на `distance_into_segment_mm` от начала
    /// сегмента — используется генератором шагов для определения текущей
    /// скорости (а значит, и периода между шагами) по мере прохождения
    /// сегмента.
    #[must_use]
    pub fn speed_at_distance(&self, distance_into_segment_mm: f32) -> f32 {
        let d = distance_into_segment_mm.max(0.0);
        if d < self.accelerate_distance_mm {
            velocity_after_distance(self.entry_speed_mm_s, self.acceleration_mm_s2, d)
        } else if d < self.accelerate_distance_mm + self.cruise_distance_mm {
            self.cruise_speed_mm_s
        } else {
            let into_decel = (d - self.accelerate_distance_mm - self.cruise_distance_mm)
                .min(self.decelerate_distance_mm);
            velocity_after_distance(self.cruise_speed_mm_s, -self.acceleration_mm_s2, into_decel)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn long_segment_reaches_full_cruise_speed() {
        let profile = TrapezoidProfile::build(1000.0, 0.0, 0.0, 100.0, 500.0);
        assert!((profile.cruise_speed_mm_s - 100.0).abs() < 1e-3);
        assert!(profile.cruise_distance_mm > 0.0);
        assert!((profile.total_distance_mm() - 1000.0).abs() < 1e-2);
    }

    #[test]
    fn short_segment_produces_triangular_profile_below_max_speed() {
        let profile = TrapezoidProfile::build(1.0, 0.0, 0.0, 100.0, 500.0);
        assert!(profile.cruise_speed_mm_s < 100.0);
        assert!((profile.cruise_distance_mm).abs() < 1e-3);
        assert!((profile.total_distance_mm() - 1.0).abs() < 1e-2);
    }

    #[test]
    fn speed_at_start_and_end_matches_entry_and_exit() {
        let profile = TrapezoidProfile::build(50.0, 10.0, 5.0, 80.0, 400.0);
        assert!((profile.speed_at_distance(0.0) - 10.0).abs() < 1e-2);
        let end_speed = profile.speed_at_distance(profile.total_distance_mm());
        assert!((end_speed - 5.0).abs() < 1e-1);
    }
}
