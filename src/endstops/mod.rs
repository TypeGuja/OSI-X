//! Концевые выключатели станка.
//!
//! [`crate::drivers::motor::axis::Axis`] уже хранит "свой" концевик и умеет
//! отвечать на вопрос "сработал ли он", но эта информация скрыта за
//! объектно-безопасным [`crate::drivers::motor::axis::AxisControl`]
//! (специально ради компактного интерфейса генератора шагов). Для `M119`
//! нужен независимый групповой опрос всех трёх концевиков сразу — за это
//! отвечает [`EndstopSet`], не зависящий от того, как организовано хранение
//! осей в `motion`/`drivers`.
//!
//! `dead_code` временно отключён: модуль полностью реализован, но будет
//! подключён к `PrinterContext::endstop_states`/`home_axes` только при
//! финальной сборке `App`.

use crate::error::{AppError, AppResult};
use crate::gcode::commands::EndstopStates;
use embedded_hal::digital::InputPin;
use std::fmt::Debug;

/// Один концевой выключатель с учётом активного уровня сигнала.
pub struct Endstop<P: InputPin> {
    pin: P,
    active_low: bool,
}

impl<P: InputPin> Endstop<P>
where
    P::Error: Debug,
{
    /// Создаёт концевик поверх уже сконфигурированного входного пина.
    /// `active_low` — `true`, если сигнал "сработал" соответствует низкому
    /// уровню (стандартно для механических микровыключателей с подтяжкой
    /// к питанию, как и остальные концевики станка — см. `board::pins`).
    #[must_use]
    pub fn new(pin: P, active_low: bool) -> Self {
        Self { pin, active_low }
    }

    /// Возвращает `true`, если концевик сработал.
    pub fn is_triggered(&mut self) -> AppResult<bool> {
        let is_low = self
            .pin
            .is_low()
            .map_err(|e| AppError::board(format!("ошибка чтения концевика: {e:?}")))?;
        Ok(if self.active_low { is_low } else { !is_low })
    }
}

/// Группа концевых выключателей всех трёх осей станка.
pub struct EndstopSet<X, Y, Z>
where
    X: InputPin,
    Y: InputPin,
    Z: InputPin,
{
    x: Endstop<X>,
    y: Endstop<Y>,
    z: Endstop<Z>,
}

impl<X, Y, Z> EndstopSet<X, Y, Z>
where
    X: InputPin,
    X::Error: Debug,
    Y: InputPin,
    Y::Error: Debug,
    Z: InputPin,
    Z::Error: Debug,
{
    /// Создаёт группу из трёх уже сконфигурированных концевиков.
    #[must_use]
    pub fn new(x: Endstop<X>, y: Endstop<Y>, z: Endstop<Z>) -> Self {
        Self { x, y, z }
    }

    /// Опрашивает все три концевика и возвращает их состояние в формате,
    /// напрямую пригодном для [`crate::gcode::commands::PrinterContext::endstop_states`]
    /// (ответ на `M119`).
    pub fn states(&mut self) -> AppResult<EndstopStates> {
        Ok(EndstopStates {
            x_triggered: self.x.is_triggered()?,
            y_triggered: self.y.is_triggered()?,
            z_triggered: self.z.is_triggered()?,
        })
    }

    /// Возвращает `true`, если сработал концевик указанной оси —
    /// используется процедурой хоуминга для остановки движения оси при
    /// достижении механического предела.
    pub fn is_axis_triggered(&mut self, axis: crate::types::AxisId) -> AppResult<bool> {
        match axis {
            crate::types::AxisId::X => self.x.is_triggered(),
            crate::types::AxisId::Y => self.y.is_triggered(),
            crate::types::AxisId::Z => self.z.is_triggered(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::convert::Infallible;

    struct FixedPin(bool);
    impl embedded_hal::digital::ErrorType for FixedPin {
        type Error = Infallible;
    }
    impl InputPin for FixedPin {
        fn is_high(&mut self) -> Result<bool, Self::Error> {
            Ok(!self.0)
        }
        fn is_low(&mut self) -> Result<bool, Self::Error> {
            Ok(self.0)
        }
    }

    #[test]
    fn active_low_endstop_reports_triggered_on_low_signal() {
        let mut endstop = Endstop::new(FixedPin(true), true);
        assert!(endstop.is_triggered().unwrap());

        let mut not_triggered = Endstop::new(FixedPin(false), true);
        assert!(!not_triggered.is_triggered().unwrap());
    }

    #[test]
    fn active_high_endstop_inverts_interpretation() {
        let mut endstop = Endstop::new(FixedPin(true), false);
        assert!(!endstop.is_triggered().unwrap());
    }

    #[test]
    fn endstop_set_reports_all_three_axes() {
        let mut set = EndstopSet::new(
            Endstop::new(FixedPin(true), true),
            Endstop::new(FixedPin(false), true),
            Endstop::new(FixedPin(false), true),
        );
        let states = set.states().unwrap();
        assert!(states.x_triggered);
        assert!(!states.y_triggered);
        assert!(!states.z_triggered);
    }
}
