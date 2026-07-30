//! Генератор шагов: исполняет один [`MotionSegment`] из очереди
//! планировщика, синхронно управляя всеми тремя осями.
//!
//! Ведущая ось (с наибольшим числом шагов в сегменте) диктует тайминг,
//! рассчитанный из [`TrapezoidProfile`]; ведомые оси синхронизируются с ней
//! классическим алгоритмом Брезенхэма, что гарантирует прямолинейность
//! траектории независимо от соотношения числа шагов между осями.

use crate::drivers::motor::axis::AxisControl;
use crate::error::AppResult;
use crate::motion::queue::MotionSegment;
use crate::motion::trapezoid::TrapezoidProfile;
use crate::types::{Microseconds, MotorDirection};

/// Источник точных микросекундных задержек между шаговыми импульсами.
///
/// Обобщён отдельным трейтом (а не завязан напрямую на
/// `esp_idf_hal::delay::Ets`), чтобы логику интерполяции можно было
/// покрыть хостовыми тестами с фиктивной реализацией, не требующей ESP-IDF.
pub trait StepClock {
    /// Блокирующе ждёт `microseconds` микросекунд.
    fn delay_us(&mut self, microseconds: u32);
}

/// Реализация [`StepClock`] на основе `esp_idf_hal::delay::Ets` —
/// busy-wait задержка с точностью до микросекунды, необходимой для
/// тайминга шаговых импульсов (штатный `vTaskDelay` FreeRTOS даёт только
/// миллисекундную гранулярность).
pub struct EtsStepClock;

impl StepClock for EtsStepClock {
    fn delay_us(&mut self, microseconds: u32) {
        esp_idf_hal::delay::Ets::delay_us(microseconds);
    }
}

/// Минимальная скорость, подставляемая вместо `0`, чтобы избежать деления
/// на ноль при расчёте периода между шагами на границах сегмента (реальная
/// скорость `0` соответствовала бы бесконечному периоду).
const MIN_SPEED_MM_S: f32 = 1.0;

/// Генератор шагов, управляющий тремя осями станка (X, Y, Z — индексы `0`,
/// `1`, `2`, соответствующие полям `a`, `b`, `c` [`crate::motion::kinematics::AxisPosition`]).
pub struct StepGenerator<C: StepClock> {
    axes: [Box<dyn AxisControl>; 3],
    steps_per_mm: [f32; 3],
    max_step_rate_hz: u32,
    clock: C,
}

impl<C: StepClock> StepGenerator<C> {
    /// Прямой доступ к управляемым осям — используется финальной сборкой
    /// `App` для `M17`/`M18` (включение/выключение моторов) и хоуминга
    /// (`G28`), которые не укладываются в узкий интерфейс исполнения
    /// сегментов [`StepGenerator::execute_segment`].
    pub fn axes_mut(&mut self) -> &mut [Box<dyn AxisControl>; 3] {
        &mut self.axes
    }

    /// Создаёт генератор шагов над тремя уже сконфигурированными осями.
    #[must_use]
    pub fn new(axes: [Box<dyn AxisControl>; 3], steps_per_mm: [f32; 3], max_step_rate_hz: u32, clock: C) -> Self {
        Self {
            axes,
            steps_per_mm,
            max_step_rate_hz,
            clock,
        }
    }

    /// Исполняет сегмент целиком (блокирующий вызов) — от текущей
    /// зафиксированной позиции осей до `segment.target_position`.
    pub fn execute_segment(&mut self, segment: &MotionSegment) -> AppResult<()> {
        let profile = TrapezoidProfile::build(
            segment.distance_mm,
            segment.entry_speed_mm_s,
            segment.exit_speed_mm_s,
            segment.max_speed_mm_s,
            segment.max_acceleration_mm_s2,
        );

        let target_steps = [
            (segment.target_position.a * self.steps_per_mm[0]).round() as i64,
            (segment.target_position.b * self.steps_per_mm[1]).round() as i64,
            (segment.target_position.c * self.steps_per_mm[2]).round() as i64,
        ];

        let mut delta = [0i64; 3];
        for i in 0..3 {
            let current = self.axes[i].position_steps();
            delta[i] = target_steps[i] - current;
            let direction = if delta[i] >= 0 {
                MotorDirection::Forward
            } else {
                MotorDirection::Backward
            };
            self.axes[i].set_direction(direction)?;
        }

        let steps_abs = [delta[0].unsigned_abs(), delta[1].unsigned_abs(), delta[2].unsigned_abs()];
        let main_axis = (0..3usize)
            .max_by_key(|&i| steps_abs[i])
            .expect("диапазон 0..3 непуст");
        let main_steps = steps_abs[main_axis];

        if main_steps == 0 {
            return Ok(());
        }

        let min_period_us = Microseconds::from_hz(self.max_step_rate_hz as f32).0;
        let main_steps_per_mm = self.steps_per_mm[main_axis].max(1e-6);

        // Накопители ошибки алгоритма Брезенхэма для ведомых осей.
        let mut error = [0i64; 3];

        for step_index in 0..main_steps {
            let distance_into_segment = segment.distance_mm * (step_index as f32 / main_steps as f32);
            let speed_mm_s = profile.speed_at_distance(distance_into_segment).max(MIN_SPEED_MM_S);
            let step_period_us = Microseconds::from_hz(speed_mm_s * main_steps_per_mm)
                .0
                .max(min_period_us)
                .min(u64::from(u32::MAX));

            for i in 0..3 {
                if i == main_axis {
                    self.axes[i].step()?;
                    continue;
                }
                if steps_abs[i] == 0 {
                    continue;
                }
                error[i] += steps_abs[i] as i64;
                if error[i] >= main_steps as i64 {
                    error[i] -= main_steps as i64;
                    self.axes[i].step()?;
                }
            }

            self.clock.delay_us(step_period_us as u32);
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::drivers::motor::driver::MotorDriver;
    use crate::motion::kinematics::AxisPosition;
    use crate::types::AxisId;
    use std::sync::{Arc, Mutex};

    /// Фиктивный (не требующий железа) драйвер для хостовых тестов
    /// генератора шагов — записывает историю направлений/шагов вместо
    /// обращения к GPIO. Использует `Arc<Mutex<..>>`, а не `Rc<RefCell<..>>`,
    /// поскольку `AxisControl` требует `Send` (генератор шагов должен уметь
    /// работать внутри выделенной задачи FreeRTOS).
    struct RecordingDriver {
        log: Arc<Mutex<Vec<(MotorDirection, i64)>>>,
        enabled: bool,
    }

    impl MotorDriver for RecordingDriver {
        fn enable(&mut self) -> AppResult<()> {
            self.enabled = true;
            Ok(())
        }
        fn disable(&mut self) -> AppResult<()> {
            self.enabled = false;
            Ok(())
        }
        fn is_enabled(&self) -> bool {
            self.enabled
        }
        fn set_direction(&mut self, direction: MotorDirection) -> AppResult<()> {
            self.log.lock().unwrap().push((direction, 0));
            Ok(())
        }
        fn step(&mut self) -> AppResult<()> {
            if let Some(last) = self.log.lock().unwrap().last_mut() {
                last.1 += 1;
            }
            Ok(())
        }
        fn set_speed(&mut self, _steps_per_second: f32) -> AppResult<()> {
            Ok(())
        }
        fn stop(&mut self) -> AppResult<()> {
            self.disable()
        }
    }

    /// Фиктивный "всегда не сработавший" концевик для тестов.
    struct AlwaysHighPin;
    impl embedded_hal::digital::ErrorType for AlwaysHighPin {
        type Error = std::convert::Infallible;
    }
    impl embedded_hal::digital::InputPin for AlwaysHighPin {
        fn is_high(&mut self) -> Result<bool, Self::Error> {
            Ok(true)
        }
        fn is_low(&mut self) -> Result<bool, Self::Error> {
            Ok(false)
        }
    }

    struct NoOpClock;
    impl StepClock for NoOpClock {
        fn delay_us(&mut self, _microseconds: u32) {}
    }

    fn make_axis(id: AxisId, log: Arc<Mutex<Vec<(MotorDirection, i64)>>>) -> Box<dyn AxisControl> {
        let driver = RecordingDriver { log, enabled: true };
        Box::new(crate::drivers::motor::axis::Axis::new(id, driver, AlwaysHighPin, false, true))
    }

    #[test]
    fn main_axis_receives_exactly_the_requested_number_of_steps() {
        let log_x = Arc::new(Mutex::new(Vec::new()));
        let log_y = Arc::new(Mutex::new(Vec::new()));
        let log_z = Arc::new(Mutex::new(Vec::new()));

        let axes: [Box<dyn AxisControl>; 3] = [
            make_axis(AxisId::X, log_x.clone()),
            make_axis(AxisId::Y, log_y.clone()),
            make_axis(AxisId::Z, log_z.clone()),
        ];

        let mut generator = StepGenerator::new(axes, [80.0, 80.0, 400.0], 200_000, NoOpClock);

        let segment = MotionSegment {
            target_position: AxisPosition { a: 10.0, b: 5.0, c: 0.0 },
            unit_vector: [0.894_427, 0.447_214, 0.0],
            distance_mm: (10.0f32 * 10.0 + 5.0 * 5.0).sqrt(),
            requested_feed_rate_mm_s: 50.0,
            max_speed_mm_s: 50.0,
            max_acceleration_mm_s2: 500.0,
            entry_speed_mm_s: 0.0,
            exit_speed_mm_s: 0.0,
        };

        generator.execute_segment(&segment).unwrap();

        let steps_x: i64 = log_x.lock().unwrap().iter().map(|(_, n)| n).sum();
        let steps_y: i64 = log_y.lock().unwrap().iter().map(|(_, n)| n).sum();
        // X — ведущая ось (10мм * 80 шаг/мм = 800 шагов > Y: 5мм*80=400 шагов).
        assert_eq!(steps_x, 800);
        // Y должна получить примерно вдвое меньше шагов (Брезенхэм даёт точное
        // распределение 400 из 800 при кратном соотношении).
        assert_eq!(steps_y, 400);
    }
}
