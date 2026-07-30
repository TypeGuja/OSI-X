//! Управление вентилятором обдува детали (`M106`/`M107`).
//!
//! Отдельный, предельно простой модуль: в отличие от нагревателя,
//! вентилятору не нужны ни термистор, ни ПИД, ни защита от thermal
//! runaway — только ШИМ-выход с сохранённым текущим значением скважности
//! (для отчётности и `M500`/`M501`, если конфигурация вентилятора когда-то
//! станет персистентной).

use crate::error::AppResult;
use crate::temperature::heater::PwmOutput;

/// Вентилятор, управляемый через ШИМ-выход `0..=255`.
pub struct Fan<P: PwmOutput> {
    pwm: P,
    current_duty: u8,
}

impl<P: PwmOutput> Fan<P> {
    /// Создаёт вентилятор в выключенном состоянии.
    pub fn new(mut pwm: P) -> AppResult<Self> {
        pwm.set_duty(0)?;
        Ok(Self { pwm, current_duty: 0 })
    }

    /// Устанавливает скорость вентилятора (`0` — выключен, `255` — полная
    /// мощность). Соответствует диапазону параметра `S` команды `M106`.
    pub fn set_speed(&mut self, duty_0_255: u8) -> AppResult<()> {
        self.pwm.set_duty(duty_0_255)?;
        self.current_duty = duty_0_255;
        Ok(())
    }

    /// Немедленно останавливает вентилятор (`M107`).
    pub fn stop(&mut self) -> AppResult<()> {
        self.set_speed(0)
    }

    /// Текущая установленная скорость.
    #[must_use]
    pub fn current_speed(&self) -> u8 {
        self.current_duty
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct RecordingPwm {
        last_duty: u8,
    }
    impl PwmOutput for RecordingPwm {
        fn set_duty(&mut self, duty_0_255: u8) -> AppResult<()> {
            self.last_duty = duty_0_255;
            Ok(())
        }
    }

    #[test]
    fn fan_starts_off() {
        let fan = Fan::new(RecordingPwm { last_duty: 200 }).unwrap();
        assert_eq!(fan.current_speed(), 0);
        assert_eq!(fan.pwm.last_duty, 0);
    }

    #[test]
    fn set_speed_updates_pwm_and_current_speed() {
        let mut fan = Fan::new(RecordingPwm { last_duty: 0 }).unwrap();
        fan.set_speed(180).unwrap();
        assert_eq!(fan.current_speed(), 180);
        assert_eq!(fan.pwm.last_duty, 180);
    }

    #[test]
    fn stop_sets_speed_to_zero() {
        let mut fan = Fan::new(RecordingPwm { last_duty: 0 }).unwrap();
        fan.set_speed(255).unwrap();
        fan.stop().unwrap();
        assert_eq!(fan.current_speed(), 0);
    }
}
