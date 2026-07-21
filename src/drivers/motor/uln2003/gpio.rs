//! Обёртка над четырьмя GPIO-выходами, управляющими обмотками через
//! драйвер ULN2003 (используется двигателем 28BYJ-48).

use crate::error::{AppError, AppResult};
use embedded_hal::digital::OutputPin;
use std::fmt::Debug;

/// Четыре GPIO-пина, соединённых с входами `IN1..IN4` микросхемы ULN2003.
///
/// Обобщена по `embedded-hal`, а не по конкретным типам `esp-idf-hal`, что
/// позволяет использовать драйвер ULN2003 без изменений при переносе на
/// другую плату или HAL.
pub struct Uln2003Pins<A, B, C, D>
where
    A: OutputPin,
    B: OutputPin,
    C: OutputPin,
    D: OutputPin,
{
    in1: A,
    in2: B,
    in3: C,
    in4: D,
}

impl<A, B, C, D> Uln2003Pins<A, B, C, D>
where
    A: OutputPin,
    A::Error: Debug,
    B: OutputPin,
    B::Error: Debug,
    C: OutputPin,
    C::Error: Debug,
    D: OutputPin,
    D::Error: Debug,
{
    /// Создаёт обёртку над четырьмя уже сконфигурированными выходными пинами.
    #[must_use]
    pub fn new(in1: A, in2: B, in3: C, in4: D) -> Self {
        Self { in1, in2, in3, in4 }
    }

    /// Применяет 4-битный паттерн (по одному биту на обмотку, начиная с
    /// `IN1` — младший бит) на все четыре пина одновременно.
    pub fn apply_pattern(&mut self, pattern: u8) -> AppResult<()> {
        self.set_pin(0, pattern & 0b0001 != 0)?;
        self.set_pin(1, pattern & 0b0010 != 0)?;
        self.set_pin(2, pattern & 0b0100 != 0)?;
        self.set_pin(3, pattern & 0b1000 != 0)?;
        Ok(())
    }

    /// Обесточивает все четыре обмотки (снимает удерживающий момент).
    pub fn de_energize(&mut self) -> AppResult<()> {
        self.apply_pattern(0)
    }

    fn set_pin(&mut self, index: u8, high: bool) -> AppResult<()> {
        let result = match index {
            0 => set_state(&mut self.in1, high),
            1 => set_state(&mut self.in2, high),
            2 => set_state(&mut self.in3, high),
            _ => set_state(&mut self.in4, high),
        };
        result.map_err(|e| AppError::motor_driver("uln2003", format!("IN{}: {e:?}", index + 1)))
    }
}

/// Устанавливает состояние произвольного `embedded-hal` выхода.
fn set_state<P: OutputPin>(pin: &mut P, high: bool) -> Result<(), P::Error> {
    if high {
        pin.set_high()
    } else {
        pin.set_low()
    }
}
