//! Обёртка [`Axis`], объединяющая произвольный [`MotorDriver`], концевой
//! выключатель и учёт позиции в шагах.
//!
//! `Axis` — единственное место, где логическое направление оси (заданное
//! конфигурацией `motion.toml`, поле `invert_direction`) преобразуется в
//! физическое направление вращения конкретного драйвера. Планировщик
//! движения оперирует только логическими направлениями и координатами в
//! шагах, не заботясь о распайке двигателя.

use crate::drivers::motor::driver::MotorDriver;
use crate::error::{AppError, AppResult};
use crate::types::{AxisId, MotorDirection};
use embedded_hal::digital::InputPin;
use std::fmt::Debug;

/// Ось станка: драйвер двигателя + концевой выключатель + учёт позиции.
///
/// Параметризована трейтами, а не конкретными типами `esp-idf-hal`, что
/// позволяет использовать `Axis` как с [`crate::drivers::motor::tmc2209::Tmc2209Driver`],
/// так и с [`crate::drivers::motor::uln2003::Uln2003Driver`] без каких-либо
/// изменений в этом файле.
pub struct Axis<D, E>
where
    D: MotorDriver,
    E: InputPin,
{
    id: AxisId,
    driver: D,
    endstop: E,
    /// Инвертировать логическое направление относительно физического
    /// вращения драйвера (значение берётся из `motion.toml`).
    invert_direction: bool,
    /// Считать концевик сработавшим при низком уровне сигнала (обычно `true`
    /// для нормально-замкнутых микровыключателей с подтяжкой к питанию).
    endstop_active_low: bool,
    /// Текущая позиция оси в шагах двигателя (без учёта `steps_per_mm` —
    /// перевод в миллиметры выполняет `motion::kinematics`).
    position_steps: i64,
    /// Последнее логическое направление, установленное вызывающим кодом.
    logical_direction: MotorDirection,
}

impl<D, E> Axis<D, E>
where
    D: MotorDriver,
    E: InputPin,
    E::Error: Debug,
{
    /// Создаёт ось на основе уже сконфигурированного драйвера и пина
    /// концевого выключателя.
    pub fn new(id: AxisId, driver: D, endstop: E, invert_direction: bool, endstop_active_low: bool) -> Self {
        Self {
            id,
            driver,
            endstop,
            invert_direction,
            endstop_active_low,
            position_steps: 0,
            logical_direction: MotorDirection::Forward,
        }
    }

    /// Идентификатор оси.
    #[must_use]
    pub fn id(&self) -> AxisId {
        self.id
    }

    /// Включает драйвер оси.
    pub fn enable(&mut self) -> AppResult<()> {
        self.driver.enable()
    }

    /// Выключает драйвер оси.
    pub fn disable(&mut self) -> AppResult<()> {
        self.driver.disable()
    }

    /// Возвращает `true`, если драйвер оси включён.
    #[must_use]
    pub fn is_enabled(&self) -> bool {
        self.driver.is_enabled()
    }

    /// Устанавливает логическое направление движения (с учётом
    /// `invert_direction` из конфигурации).
    pub fn set_direction(&mut self, direction: MotorDirection) -> AppResult<()> {
        self.logical_direction = direction;
        let physical = if self.invert_direction {
            direction.reversed()
        } else {
            direction
        };
        self.driver.set_direction(physical)
    }

    /// Выполняет один шаг в ранее установленном логическом направлении и
    /// обновляет счётчик позиции.
    pub fn step(&mut self) -> AppResult<()> {
        self.driver.step()?;
        match self.logical_direction {
            MotorDirection::Forward => self.position_steps += 1,
            MotorDirection::Backward => self.position_steps -= 1,
        }
        Ok(())
    }

    /// Сообщает драйверу целевую скорость (шагов в секунду).
    pub fn set_speed(&mut self, steps_per_second: f32) -> AppResult<()> {
        self.driver.set_speed(steps_per_second)
    }

    /// Немедленно останавливает ось (безопасна для вызова из E-Stop).
    pub fn stop(&mut self) -> AppResult<()> {
        self.driver.stop()
    }

    /// Текущая позиция оси в шагах двигателя.
    #[must_use]
    pub fn position_steps(&self) -> i64 {
        self.position_steps
    }

    /// Принудительно устанавливает позицию оси (используется после
    /// хоуминга или `G92`).
    pub fn reset_position(&mut self, steps: i64) {
        self.position_steps = steps;
    }

    /// Возвращает `true`, если концевой выключатель оси сработал.
    pub fn is_endstop_triggered(&mut self) -> AppResult<bool> {
        let is_low = self
            .endstop
            .is_low()
            .map_err(|e| AppError::motor_driver("axis-endstop", format!("{e:?}")))?;
        Ok(if self.endstop_active_low { is_low } else { !is_low })
    }
}

/// Объектно-безопасное подмножество операций [`Axis`], не зависящее от
/// конкретных типов драйвера/концевика.
///
/// [`crate::motion::step_generator::StepGenerator`] управляет тремя осями с
/// разными физическими драйверами (TMC2209 на X/Y, ULN2003 на Z) через один
/// массив `[Box<dyn AxisControl>; 3]` — без этого трейта пришлось бы либо
/// параметризовать генератор шагов шестью типовыми параметрами (по два на
/// ось), либо привязываться к конкретной комбинации драйверов, что
/// противоречит требованию ТЗ о независимости от типа драйвера оси Z.
pub trait AxisControl: Send {
    /// Идентификатор оси.
    fn id(&self) -> AxisId;
    /// См. [`Axis::enable`].
    fn enable(&mut self) -> AppResult<()>;
    /// См. [`Axis::disable`].
    fn disable(&mut self) -> AppResult<()>;
    /// См. [`Axis::set_direction`].
    fn set_direction(&mut self, direction: MotorDirection) -> AppResult<()>;
    /// См. [`Axis::step`].
    fn step(&mut self) -> AppResult<()>;
    /// См. [`Axis::stop`].
    fn stop(&mut self) -> AppResult<()>;
    /// См. [`Axis::position_steps`].
    fn position_steps(&self) -> i64;
    /// См. [`Axis::reset_position`].
    fn reset_position(&mut self, steps: i64);
}

impl<D, E> AxisControl for Axis<D, E>
where
    D: MotorDriver + Send,
    E: InputPin + Send,
    E::Error: Debug,
{
    fn id(&self) -> AxisId {
        Axis::id(self)
    }

    fn enable(&mut self) -> AppResult<()> {
        Axis::enable(self)
    }

    fn disable(&mut self) -> AppResult<()> {
        Axis::disable(self)
    }

    fn set_direction(&mut self, direction: MotorDirection) -> AppResult<()> {
        Axis::set_direction(self, direction)
    }

    fn step(&mut self) -> AppResult<()> {
        Axis::step(self)
    }

    fn stop(&mut self) -> AppResult<()> {
        Axis::stop(self)
    }

    fn position_steps(&self) -> i64 {
        Axis::position_steps(self)
    }

    fn reset_position(&mut self, steps: i64) {
        Axis::reset_position(self, steps)
    }
}
