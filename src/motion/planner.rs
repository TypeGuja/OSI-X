//! Планировщик движения: приём целевых точек, кинематическое преобразование,
//! постановка в очередь и look-ahead пересчёт допустимых скоростей на
//! стыках сегментов (junction deviation + jerk).

use crate::config::motion::MotionConfig;
use crate::error::{AppError, AppResult};
use crate::motion::kinematics::{CartesianPosition, Kinematics};
use crate::motion::queue::{MotionQueue, MotionSegment};
use crate::types::AxisId;

/// Минимальная длина сегмента, при которой ещё имеет смысл вычислять
/// единичный вектор направления (короче — сегмент считается вырожденным и
/// отклоняется, чтобы не делить на ноль при нормализации).
const MIN_SEGMENT_LENGTH_MM: f32 = 1e-5;

/// Планировщик движения станка.
///
/// Владеет очередью сегментов, конкретной кинематикой станка и текущим
/// известным положением эффектора. Не знает о шаговых двигателях —
/// генератор шагов ([`crate::motion::step_generator::StepGenerator`])
/// потребляет сегменты из очереди независимо.
pub struct MotionPlanner {
    kinematics: Box<dyn Kinematics>,
    queue: MotionQueue,
    current_position: CartesianPosition,
    junction_deviation_mm: f32,
    /// Ограничения по осям в порядке `AxisId::ALL` (X, Y, Z), взятые из
    /// `motion.toml` при создании планировщика.
    axis_limits: [AxisLimits; 3],
}

/// Ограничения одной логической оси, извлечённые из конфигурации.
#[derive(Debug, Clone, Copy)]
struct AxisLimits {
    max_speed_mm_s: f32,
    max_acceleration_mm_s2: f32,
    jerk_mm_s: f32,
}

impl MotionPlanner {
    /// Создаёт планировщик на основе конфигурации движения и выбранной
    /// кинематики, с эффектором в начале координат.
    pub fn new(config: &MotionConfig, kinematics: Box<dyn Kinematics>) -> AppResult<Self> {
        let mut axis_limits = [AxisLimits {
            max_speed_mm_s: 0.0,
            max_acceleration_mm_s2: 0.0,
            jerk_mm_s: 0.0,
        }; 3];

        for (i, axis_id) in AxisId::ALL.iter().enumerate() {
            let cfg = config.axis(*axis_id).ok_or_else(|| {
                AppError::config(
                    "motion.toml",
                    format!("отсутствует конфигурация оси {axis_id}"),
                )
            })?;
            axis_limits[i] = AxisLimits {
                max_speed_mm_s: cfg.max_speed_mm_s,
                max_acceleration_mm_s2: cfg.max_acceleration_mm_s2,
                jerk_mm_s: cfg.jerk_mm_s,
            };
        }

        Ok(Self {
            kinematics,
            queue: MotionQueue::new(config.planner_queue_depth),
            current_position: CartesianPosition { x: 0.0, y: 0.0, z: 0.0 },
            junction_deviation_mm: config.junction_deviation.deviation_mm,
            axis_limits,
        })
    }

    /// Текущее известное положение эффектора.
    #[must_use]
    pub fn current_position(&self) -> CartesianPosition {
        self.current_position
    }

    /// Принудительно устанавливает текущее положение без движения
    /// (используется после хоуминга и для обработки `G92`).
    pub fn set_current_position(&mut self, position: CartesianPosition) {
        self.current_position = position;
    }

    /// Возвращает `true`, если очередь планировщика заполнена и не может
    /// принять новый сегмент прямо сейчас.
    #[must_use]
    pub fn is_queue_full(&self) -> bool {
        self.queue.is_full()
    }

    /// Планирует линейное перемещение к абсолютной точке `target` с
    /// запрошенной скоростью подачи `feed_rate_mm_s`.
    ///
    /// Возвращает `Ok(false)`, если очередь заполнена (сегмент не был
    /// добавлен — вызывающий код должен подождать и повторить), либо
    /// `Ok(true)` при успешной постановке в очередь и пересчёте look-ahead.
    pub fn plan_linear_move(&mut self, target: CartesianPosition, feed_rate_mm_s: f32) -> AppResult<bool> {
        if self.queue.is_full() {
            return Ok(false);
        }

        let start_motor = self.kinematics.cartesian_to_motor(self.current_position)?;
        let target_motor = self.kinematics.cartesian_to_motor(target)?;

        let delta = [
            target_motor.a - start_motor.a,
            target_motor.b - start_motor.b,
            target_motor.c - start_motor.c,
        ];
        let distance_mm = (delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]).sqrt();

        if distance_mm < MIN_SEGMENT_LENGTH_MM {
            // Нулевое перемещение (например, дублирующий G1 без изменения
            // координат) — не ошибка, просто нечего планировать.
            self.current_position = target;
            return Ok(true);
        }

        let unit_vector = [delta[0] / distance_mm, delta[1] / distance_mm, delta[2] / distance_mm];

        let mut max_speed = feed_rate_mm_s.max(0.0);
        let mut max_acceleration = f32::MAX;
        for (i, limits) in self.axis_limits.iter().enumerate() {
            let component = unit_vector[i].abs();
            if component > f32::EPSILON {
                max_speed = max_speed.min(limits.max_speed_mm_s / component);
                max_acceleration = max_acceleration.min(limits.max_acceleration_mm_s2 / component);
            }
        }

        let segment = MotionSegment {
            target_position: target_motor,
            unit_vector,
            distance_mm,
            requested_feed_rate_mm_s: feed_rate_mm_s,
            max_speed_mm_s: max_speed,
            max_acceleration_mm_s2: max_acceleration,
            entry_speed_mm_s: 0.0,
            exit_speed_mm_s: 0.0,
        };

        if !self.queue.push(segment) {
            return Ok(false);
        }

        self.current_position = target;
        self.recalculate();
        Ok(true)
    }

    /// Извлекает следующий готовый к исполнению сегмент (с уже
    /// пересчитанными скоростями входа/выхода).
    pub fn pop_next_segment(&mut self) -> Option<MotionSegment> {
        self.queue.pop_front()
    }

    /// Количество сегментов, ожидающих исполнения.
    #[must_use]
    pub fn queued_len(&self) -> usize {
        self.queue.len()
    }

    /// Пересчитывает допустимые скорости на границах всех сегментов в
    /// очереди (алгоритм look-ahead: обратный проход учитывает
    /// junction deviation + jerk и ограничение торможением, прямой проход —
    /// ограничение разгоном). Консервативно предполагает, что очередь
    /// начинается и заканчивается в состоянии покоя — по мере поступления
    /// новых сегментов более ранние получают возможность двигаться быстрее.
    fn recalculate(&mut self) {
        let count = self.queue.len();
        if count == 0 {
            return;
        }

        // `boundary[i]` — скорость на границе ПЕРЕД сегментом `i` (индексация
        // 0..=count, где `boundary[0]` и `boundary[count]` всегда 0 —
        // консервативное предположение "начинаем и заканчиваем из покоя").
        let mut boundary = vec![0.0f32; count + 1];

        {
            let segments: Vec<MotionSegment> = self.queue.iter().copied().collect();

            // Начальная оценка границ по junction deviation + jerk между
            // соседними сегментами.
            for i in 1..count {
                let prev = &segments[i - 1];
                let next = &segments[i];
                let junction_limit = junction_speed(
                    prev.unit_vector,
                    next.unit_vector,
                    self.junction_deviation_mm,
                    prev.max_acceleration_mm_s2.min(next.max_acceleration_mm_s2),
                    prev.max_speed_mm_s.min(next.max_speed_mm_s),
                );
                let jerk_limit = jerk_limited_speed(
                    prev.unit_vector,
                    next.unit_vector,
                    self.jerk_limit_for(prev.unit_vector, next.unit_vector),
                    junction_limit,
                );
                boundary[i] = jerk_limit;
            }

            // Обратный проход: ограничение возможностью затормозить с
            // текущей границы до следующей на дистанции сегмента.
            for i in (0..count).rev() {
                let seg = &segments[i];
                let limited = velocity_reachable(boundary[i + 1], seg.max_acceleration_mm_s2, seg.distance_mm);
                boundary[i] = boundary[i].min(limited);
            }

            // Прямой проход: ограничение возможностью разогнаться с
            // предыдущей границы до следующей на дистанции сегмента.
            for i in 0..count {
                let seg = &segments[i];
                let limited = velocity_reachable(boundary[i], seg.max_acceleration_mm_s2, seg.distance_mm);
                boundary[i + 1] = boundary[i + 1].min(limited);
            }
        }

        for (i, segment) in self.queue.iter_mut().enumerate() {
            segment.entry_speed_mm_s = boundary[i].min(segment.max_speed_mm_s);
            segment.exit_speed_mm_s = boundary[i + 1].min(segment.max_speed_mm_s);
        }
    }

    /// Минимальный jerk среди осей, реально участвующих в переходе между
    /// двумя направлениями (используется как компромисс: берём наиболее
    /// строгое из ограничений затронутых осей).
    fn jerk_limit_for(&self, prev_unit: [f32; 3], next_unit: [f32; 3]) -> f32 {
        let mut limit = f32::MAX;
        for (i, limits) in self.axis_limits.iter().enumerate() {
            if prev_unit[i].abs() > f32::EPSILON || next_unit[i].abs() > f32::EPSILON {
                limit = limit.min(limits.jerk_mm_s);
            }
        }
        if limit.is_finite() {
            limit
        } else {
            0.0
        }
    }
}

/// Максимальная скорость прохождения стыка между сегментами `prev` → `next`
/// по формуле junction deviation (используется в Grbl):
///
/// `sin(θ/2) = sqrt((1 - cosθ) / 2)`, `radius = jd * sin(θ/2) / (1 - sin(θ/2))`,
/// `v_junction = sqrt(radius * acceleration)`, где `θ` — угол между
/// направлением движения на выходе из `prev` и входом в `next` (`0` —
/// прямая линия без замедления, `π` — разворот на месте, требующий полной
/// остановки).
fn junction_speed(
    prev_unit: [f32; 3],
    next_unit: [f32; 3],
    junction_deviation_mm: f32,
    acceleration_mm_s2: f32,
    max_speed_mm_s: f32,
) -> f32 {
    let cos_theta = -(prev_unit[0] * next_unit[0] + prev_unit[1] * next_unit[1] + prev_unit[2] * next_unit[2]);

    if cos_theta > 0.999_999 {
        // Разворот на месте (движение назад по той же прямой) — требуется
        // полная остановка.
        return 0.0;
    }

    if cos_theta < -0.999_999 {
        // Прямая линия без изменения направления — стык не ограничивает
        // скорость сверх обычных лимитов сегментов.
        return max_speed_mm_s;
    }

    let sin_half_theta = ((1.0 - cos_theta) / 2.0).max(0.0).sqrt();
    if sin_half_theta < 1e-6 {
        return max_speed_mm_s;
    }
    if (1.0 - sin_half_theta).abs() < 1e-6 {
        return 0.0;
    }

    let radius = junction_deviation_mm * sin_half_theta / (1.0 - sin_half_theta);
    (radius * acceleration_mm_s2).sqrt().min(max_speed_mm_s)
}

/// Дополнительно ограничивает скорость стыка так, чтобы изменение скорости
/// по каждой затронутой оси не превышало настроенный jerk — классическое
/// ограничение Marlin, применяемое поверх junction deviation.
fn jerk_limited_speed(prev_unit: [f32; 3], next_unit: [f32; 3], jerk_mm_s: f32, candidate_speed: f32) -> f32 {
    if candidate_speed <= 0.0 || jerk_mm_s <= 0.0 {
        return candidate_speed;
    }

    let mut speed = candidate_speed;
    for axis in 0..3 {
        let delta_component = (next_unit[axis] - prev_unit[axis]).abs();
        if delta_component < f32::EPSILON {
            continue;
        }
        let max_speed_for_axis_jerk = jerk_mm_s / delta_component;
        speed = speed.min(max_speed_for_axis_jerk);
    }
    speed
}

/// Максимальная скорость, достижимая при разгоне/торможении от `boundary`
/// на дистанции `distance_mm` с ускорением `acceleration_mm_s2`.
fn velocity_reachable(boundary: f32, acceleration_mm_s2: f32, distance_mm: f32) -> f32 {
    let reachable_sq = boundary * boundary + 2.0 * acceleration_mm_s2 * distance_mm;
    if reachable_sq <= 0.0 {
        0.0
    } else {
        reachable_sq.sqrt()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::motion::{AxisMotionConfig, JunctionDeviationConfig, MotionConfig};
    use crate::motion::kinematics::CartesianKinematics;
    use std::collections::BTreeMap;

    fn test_motion_config() -> MotionConfig {
        let mut axes = BTreeMap::new();
        let axis_cfg = AxisMotionConfig {
            steps_per_mm: 80.0,
            max_speed_mm_s: 200.0,
            max_acceleration_mm_s2: 1000.0,
            jerk_mm_s: 10.0,
            invert_direction: false,
            min_position_mm: 0.0,
            max_position_mm: 300.0,
        };
        axes.insert(AxisId::X.to_string(), axis_cfg);
        axes.insert(AxisId::Y.to_string(), axis_cfg);
        axes.insert(AxisId::Z.to_string(), axis_cfg);

        MotionConfig {
            axes,
            junction_deviation: JunctionDeviationConfig { deviation_mm: 0.02 },
            planner_queue_depth: 8,
            max_step_rate_hz: 200_000,
        }
    }

    #[test]
    fn straight_line_segments_reach_full_speed_at_shared_boundary() {
        let config = test_motion_config();
        let mut planner = MotionPlanner::new(&config, Box::new(CartesianKinematics)).unwrap();

        planner
            .plan_linear_move(CartesianPosition { x: 100.0, y: 0.0, z: 0.0 }, 150.0)
            .unwrap();
        planner
            .plan_linear_move(CartesianPosition { x: 200.0, y: 0.0, z: 0.0 }, 150.0)
            .unwrap();

        let first = planner.pop_next_segment().unwrap();
        let second = planner.pop_next_segment().unwrap();

        // Стык между двумя коллинеарными сегментами не должен требовать
        // остановки — скорость выхода первого совпадает со скоростью входа
        // второго и близка к запрошенной скорости подачи.
        assert!((first.exit_speed_mm_s - second.entry_speed_mm_s).abs() < 1e-3);
        assert!(first.exit_speed_mm_s > 50.0, "ожидался разгон на прямом участке");
    }

    #[test]
    fn sharp_reversal_forces_full_stop_at_junction() {
        let config = test_motion_config();
        let mut planner = MotionPlanner::new(&config, Box::new(CartesianKinematics)).unwrap();

        planner
            .plan_linear_move(CartesianPosition { x: 100.0, y: 0.0, z: 0.0 }, 150.0)
            .unwrap();
        planner
            .plan_linear_move(CartesianPosition { x: 0.0, y: 0.0, z: 0.0 }, 150.0)
            .unwrap();

        let first = planner.pop_next_segment().unwrap();
        assert!(first.exit_speed_mm_s.abs() < 1e-2, "разворот на 180° должен требовать остановки");
    }

    #[test]
    fn queue_rejects_push_when_full() {
        let mut config = test_motion_config();
        config.planner_queue_depth = 1;
        let mut planner = MotionPlanner::new(&config, Box::new(CartesianKinematics)).unwrap();

        assert!(planner
            .plan_linear_move(CartesianPosition { x: 10.0, y: 0.0, z: 0.0 }, 100.0)
            .unwrap());
        assert!(!planner
            .plan_linear_move(CartesianPosition { x: 20.0, y: 0.0, z: 0.0 }, 100.0)
            .unwrap());
    }
}
