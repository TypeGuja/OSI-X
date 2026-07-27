//! Базовые формулы равноускоренного движения, используемые
//! [`crate::motion::trapezoid`] и [`crate::motion::planner`].
//!
//! Вынесены в отдельный модуль, чтобы формулы не дублировались и не
//! обрастали побочными деталями (юниты, конфигурация) — здесь только
//! чистые функции над `f32`.

/// Расстояние, пройденное за время `t` при начальной скорости `v0` и
/// постоянном ускорении `a`: `d = v0*t + 0.5*a*t²`.
#[must_use]
pub fn distance(v0: f32, a: f32, t: f32) -> f32 {
    v0 * t + 0.5 * a * t * t
}

/// Скорость после прохождения расстояния `d` при начальной скорости `v0`
/// и постоянном ускорении `a`: `v = sqrt(v0² + 2*a*d)`.
///
/// Возвращает `v0`, если `v0² + 2*a*d` отрицательно (численная защита от
/// ошибок округления при `d ≈ 0`).
#[must_use]
pub fn velocity_after_distance(v0: f32, a: f32, d: f32) -> f32 {
    let under_sqrt = v0 * v0 + 2.0 * a * d;
    if under_sqrt <= 0.0 {
        0.0
    } else {
        under_sqrt.sqrt()
    }
}

/// Расстояние, необходимое для изменения скорости с `v0` до `v1` при
/// постоянном ускорении `a`: `d = (v1² - v0²) / (2*a)`.
///
/// Возвращает `0.0`, если `a` равно нулю (нет ускорения — расстояние для
/// изменения скорости не определено, а не бесконечно).
#[must_use]
pub fn distance_for_speed_change(v0: f32, v1: f32, a: f32) -> f32 {
    if a.abs() < f32::EPSILON {
        0.0
    } else {
        (v1 * v1 - v0 * v0) / (2.0 * a)
    }
}

/// Время, необходимое для изменения скорости с `v0` до `v1` при постоянном
/// ускорении `a`: `t = (v1 - v0) / a`.
#[must_use]
pub fn time_for_speed_change(v0: f32, v1: f32, a: f32) -> f32 {
    if a.abs() < f32::EPSILON {
        0.0
    } else {
        (v1 - v0) / a
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn distance_matches_uniform_motion_when_acceleration_is_zero() {
        assert!((distance(10.0, 0.0, 2.0) - 20.0).abs() < 1e-6);
    }

    #[test]
    fn velocity_after_distance_matches_known_case() {
        // v0=0, a=2, d=25 => v = sqrt(100) = 10
        let v = velocity_after_distance(0.0, 2.0, 25.0);
        assert!((v - 10.0).abs() < 1e-4);
    }

    #[test]
    fn distance_for_speed_change_matches_known_case() {
        // v0=0, v1=10, a=2 => d = 100/4 = 25
        let d = distance_for_speed_change(0.0, 10.0, 2.0);
        assert!((d - 25.0).abs() < 1e-4);
    }

    #[test]
    fn round_trip_distance_and_velocity_are_consistent() {
        let (v0, a, d) = (5.0, 3.0, 12.0);
        let v1 = velocity_after_distance(v0, a, d);
        let d_back = distance_for_speed_change(v0, v1, a);
        assert!((d_back - d).abs() < 1e-3);
    }
}
