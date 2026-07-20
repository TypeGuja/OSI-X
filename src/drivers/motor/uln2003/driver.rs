//! Драйвер [`Uln2003Driver`] — программное управление фазами
//! биполярного/униполярного шагового двигателя 28BYJ-48 через ULN2003.
//!
//! В отличие от TMC2209, у ULN2003 нет входа STEP/DIR — каждый "шаг"
//! представляет собой переключение обмоток на следующую комбинацию из
//! таблицы последовательности. Именно поэтому [`MotorDriver::step`] здесь
//! не генерирует электрический импульс, а прикладывает следующий паттерн
//! из [`FULL_STEP_SEQUENCE`] или [`HALF_STEP_SEQUENCE`].

use super::gpio::Uln2003Pins;
use crate::drivers::motor::driver::MotorDriver;
use crate::error::AppResult;
use crate::types::MotorDirection;
use embedded_hal::digital::OutputPin;
use std::fmt::Debug;

/// Полношаговая последовательность (2 обмотки одновременно активны на
/// каждом шаге — максимальный момент за счёт удвоенного числа активных
/// катушек, ценой вдвое большего числа "шагов" на оборот, чем физических
/// полушагов).
const FULL_STEP_SEQUENCE: [u8; 4] = [0b0011, 0b0110, 0b1100, 0b1001];

/// Полушаговая последовательность (8 состояний, чередование одной и двух
/// активных обмоток — вдвое более плавное вращение при том же токе).
const HALF_STEP_SEQUENCE: [u8; 8] = [
    0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001,
];

/// Режим шага двигателя ULN2003/28BYJ-48.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepMode {
    /// Полный шаг (4 состояния на цикл, больший момент).
    Full,
    /// Полушаг (8 состояний на цикл, более плавное вращение).
    Half,
}

impl StepMode {
    /// Возвращает используемую таблицу паттернов для режима.
    fn sequence(self) -> &'static [u8] {
        match self {
            StepMode::Full => &FULL_STEP_SEQUENCE,
            StepMode::Half => &HALF_STEP_SEQUENCE,
        }
    }
}

/// Драйвер двигателя 28BYJ-48 на базе ULN2003 с программным управлением
/// фазами через 4 GPIO.
pub struct Uln2003Driver<A, B, C, D>
where
    A: OutputPin,
    B: OutputPin,
    C: OutputPin,
    D: OutputPin,
{
    pins: Uln2003Pins<A, B, C, D>,
    mode: StepMode,
    sequence_index: usize,
    direction: MotorDirection,
    enabled: bool,
}

impl<A, B, C, D> Uln2003Driver<A, B, C, D>
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
    /// Создаёт драйвер в выключенном состоянии (обмотки обесточены).
    pub fn new(pins: Uln2003Pins<A, B, C, D>, mode: StepMode) -> AppResult<Self> {
        let mut driver = Self {
            pins,
            mode,
            sequence_index: 0,
            direction: MotorDirection::Forward,
            enabled: false,
        };
        driver.pins.de_energize()?;
        Ok(driver)
    }

    /// Меняет режим шага (полный/полушаг). Применяется со следующего вызова
    /// [`MotorDriver::step`] — счётчик последовательности сбрасывается на
    /// первое состояние новой таблицы во избежание рассинхронизации фаз.
    pub fn set_step_mode(&mut self, mode: StepMode) {
        self.mode = mode;
        self.sequence_index = 0;
    }

    /// Текущий установленный режим шага.
    #[must_use]
    pub fn step_mode(&self) -> StepMode {
        self.mode
    }
}

impl<A, B, C, D> MotorDriver for Uln2003Driver<A, B, C, D>
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
    fn enable(&mut self) -> AppResult<()> {
        // У ULN2003 нет отдельного силового каскада, отключаемого без
        // потери фазового состояния — "включение" означает лишь разрешение
        // прикладывать паттерны в `step()`. Обмотки запитываются по факту
        // первого вызова `step()`.
        self.enabled = true;
        Ok(())
    }

    fn disable(&mut self) -> AppResult<()> {
        self.enabled = false;
        self.pins.de_energize()
    }

    fn is_enabled(&self) -> bool {
        self.enabled
    }

    fn set_direction(&mut self, direction: MotorDirection) -> AppResult<()> {
        self.direction = direction;
        Ok(())
    }

    fn step(&mut self) -> AppResult<()> {
        if !self.enabled {
            return Ok(());
        }

        let sequence = self.mode.sequence();
        let len = sequence.len();

        self.sequence_index = match self.direction {
            MotorDirection::Forward => (self.sequence_index + 1) % len,
            MotorDirection::Backward => (self.sequence_index + len - 1) % len,
        };

        self.pins.apply_pattern(sequence[self.sequence_index])
    }

    fn set_speed(&mut self, _steps_per_second: f32) -> AppResult<()> {
        // Информационное значение: тайминг фаз задаёт `motion::step_generator`,
        // как и для TMC2209 — см. `Tmc2209Driver::set_speed`.
        Ok(())
    }

    fn stop(&mut self) -> AppResult<()> {
        self.disable()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn full_step_sequence_has_four_distinct_two_bit_patterns() {
        let unique: std::collections::HashSet<_> = FULL_STEP_SEQUENCE.iter().collect();
        assert_eq!(unique.len(), 4);
        for pattern in FULL_STEP_SEQUENCE {
            assert_eq!(pattern.count_ones(), 2, "полный шаг держит 2 обмотки активными");
        }
    }

    #[test]
    fn half_step_sequence_has_eight_distinct_patterns() {
        let unique: std::collections::HashSet<_> = HALF_STEP_SEQUENCE.iter().collect();
        assert_eq!(unique.len(), 8);
    }

    #[test]
    fn step_mode_selects_correct_sequence_length() {
        assert_eq!(StepMode::Full.sequence().len(), 4);
        assert_eq!(StepMode::Half.sequence().len(), 8);
    }
}
