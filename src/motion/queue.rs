//! Очередь сегментов движения ([`MotionQueue`]) и описание одного сегмента
//! ([`MotionSegment`]).
//!
//! Очередь хранит уже линеаризованные (после кинематики) отрезки пути в
//! пространстве логических осей и служит буфером для алгоритма look-ahead
//! ([`crate::motion::planner`]), который на каждой вставке пересчитывает
//! допустимые скорости входа/выхода для всех сегментов в очереди.

use crate::motion::kinematics::AxisPosition;
use std::collections::VecDeque;

/// Один линейный сегмент движения между двумя точками в пространстве
/// логических осей.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotionSegment {
    /// Конечное положение сегмента (абсолютное, логические оси, мм).
    pub target_position: AxisPosition,
    /// Единичный вектор направления сегмента (для расчёта угла в стыке).
    pub unit_vector: [f32; 3],
    /// Длина сегмента, мм.
    pub distance_mm: f32,
    /// Запрошенная скорость подачи (`F` из G-Code), мм/с.
    pub requested_feed_rate_mm_s: f32,
    /// Максимальная скорость сегмента с учётом ограничений участвующих
    /// осей (наименьшая из `max_speed_mm_s` осей, умноженная на компоненту
    /// единичного вектора — уже готовое ограничение для данного сегмента).
    pub max_speed_mm_s: f32,
    /// Ограничение ускорения сегмента (наименьшее среди участвующих осей).
    pub max_acceleration_mm_s2: f32,
    /// Скорость на входе в сегмент, мм/с (заполняется look-ahead).
    pub entry_speed_mm_s: f32,
    /// Скорость на выходе из сегмента, мм/с (заполняется look-ahead).
    pub exit_speed_mm_s: f32,
}

/// Очередь сегментов движения с фиксированной максимальной ёмкостью
/// (`planner_queue_depth` из `motion.toml`).
pub struct MotionQueue {
    segments: VecDeque<MotionSegment>,
    capacity: usize,
}

impl MotionQueue {
    /// Создаёт пустую очередь заданной ёмкости.
    #[must_use]
    pub fn new(capacity: usize) -> Self {
        Self {
            segments: VecDeque::with_capacity(capacity),
            capacity,
        }
    }

    /// Добавляет сегмент в конец очереди. Возвращает `false`, если очередь
    /// заполнена — вызывающий код (`gcode::executor`) должен подождать и
    /// повторить попытку, не блокируя остальную прошивку.
    #[must_use]
    pub fn push(&mut self, segment: MotionSegment) -> bool {
        if self.segments.len() >= self.capacity {
            return false;
        }
        self.segments.push_back(segment);
        true
    }

    /// Извлекает и удаляет сегмент из начала очереди (готов к исполнению
    /// генератором шагов).
    pub fn pop_front(&mut self) -> Option<MotionSegment> {
        self.segments.pop_front()
    }

    /// Возвращает `true`, если в очереди нет сегментов.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.segments.is_empty()
    }

    /// Возвращает `true`, если очередь заполнена до предела ёмкости.
    #[must_use]
    pub fn is_full(&self) -> bool {
        self.segments.len() >= self.capacity
    }

    /// Текущее количество сегментов в очереди.
    #[must_use]
    pub fn len(&self) -> usize {
        self.segments.len()
    }

    /// Настроенная максимальная ёмкость очереди.
    #[must_use]
    pub fn capacity(&self) -> usize {
        self.capacity
    }

    /// Изменяемый доступ к сегментам по индексу (`0` — самый старый,
    /// первый на исполнение) — используется алгоритмом look-ahead для
    /// пересчёта скоростей во всей очереди.
    pub fn iter_mut(&mut self) -> impl DoubleEndedIterator<Item = &mut MotionSegment> {
        self.segments.iter_mut()
    }

    /// Неизменяемый доступ к сегментам по индексу.
    pub fn iter(&self) -> impl DoubleEndedIterator<Item = &MotionSegment> {
        self.segments.iter()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_segment() -> MotionSegment {
        MotionSegment {
            target_position: AxisPosition { a: 10.0, b: 0.0, c: 0.0 },
            unit_vector: [1.0, 0.0, 0.0],
            distance_mm: 10.0,
            requested_feed_rate_mm_s: 50.0,
            max_speed_mm_s: 50.0,
            max_acceleration_mm_s2: 500.0,
            entry_speed_mm_s: 0.0,
            exit_speed_mm_s: 0.0,
        }
    }

    #[test]
    fn queue_respects_capacity() {
        let mut queue = MotionQueue::new(2);
        assert!(queue.push(sample_segment()));
        assert!(queue.push(sample_segment()));
        assert!(!queue.push(sample_segment()));
        assert!(queue.is_full());
        assert_eq!(queue.len(), 2);
    }

    #[test]
    fn pop_front_returns_oldest_segment_first() {
        let mut queue = MotionQueue::new(4);
        let mut first = sample_segment();
        first.distance_mm = 1.0;
        let mut second = sample_segment();
        second.distance_mm = 2.0;

        queue.push(first);
        queue.push(second);

        assert_eq!(queue.pop_front().unwrap().distance_mm, 1.0);
        assert_eq!(queue.pop_front().unwrap().distance_mm, 2.0);
        assert!(queue.is_empty());
    }
}
