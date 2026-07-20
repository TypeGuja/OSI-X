//! Диагностическое состояние драйвера TMC2209 (регистр `DRV_STATUS`).

use std::fmt;

/// Битовые позиции полей регистра `DRV_STATUS`.
mod bits {
    pub const OTPW: u32 = 0;
    pub const OT: u32 = 1;
    pub const S2GA: u32 = 2;
    pub const S2GB: u32 = 3;
    pub const S2VSA: u32 = 4;
    pub const S2VSB: u32 = 5;
    pub const OLA: u32 = 6;
    pub const OLB: u32 = 7;
    pub const T120: u32 = 8;
    pub const T143: u32 = 9;
    pub const T150: u32 = 10;
    pub const T157: u32 = 11;
    pub const CS_ACTUAL_SHIFT: u32 = 16;
    pub const CS_ACTUAL_MASK: u32 = 0b1_1111;
    pub const STEALTH: u32 = 30;
    pub const STST: u32 = 31;
}

/// Разобранное состояние диагностического регистра `DRV_STATUS`.
///
/// Каждое поле — независимый диагностический признак; интерпретация и
/// решения о реакции (например, отключение оси при `s2ga`/`s2gb`) находятся
/// на стороне вызывающего кода (`drivers::motor::tmc2209::Tmc2209Driver`
/// возвращает "сырую" структуру, а не принимает решения самостоятельно).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DriverStatus {
    /// Предупреждение о перегреве (свыше ~120°C).
    pub overtemperature_prewarning: bool,
    /// Аварийное отключение по перегреву.
    pub overtemperature_shutdown: bool,
    /// Короткое замыкание на землю, обмотка A.
    pub short_to_ground_a: bool,
    /// Короткое замыкание на землю, обмотка B.
    pub short_to_ground_b: bool,
    /// Короткое замыкание на питание (low-side), обмотка A.
    pub short_to_supply_a: bool,
    /// Короткое замыкание на питание (low-side), обмотка B.
    pub short_to_supply_b: bool,
    /// Обрыв обмотки A (детектируется только в состоянии покоя).
    pub open_load_a: bool,
    /// Обрыв обмотки B (детектируется только в состоянии покоя).
    pub open_load_b: bool,
    /// Температура превысила 120°C.
    pub temperature_above_120c: bool,
    /// Температура превысила 143°C.
    pub temperature_above_143c: bool,
    /// Температура превысила 150°C.
    pub temperature_above_150c: bool,
    /// Температура превысила 157°C.
    pub temperature_above_157c: bool,
    /// Фактическое значение шкалы тока (`CS_ACTUAL`), `0..=31`.
    pub current_scale_actual: u8,
    /// Активен режим StealthChop (в противовес SpreadCycle).
    pub stealth_chop_active: bool,
    /// Двигатель находится в состоянии покоя (нет вращения).
    pub standstill: bool,
}

impl DriverStatus {
    /// Разбирает 32-битное значение регистра `DRV_STATUS`.
    #[must_use]
    pub fn from_u32(value: u32) -> Self {
        let bit = |pos: u32| value & (1 << pos) != 0;
        Self {
            overtemperature_prewarning: bit(bits::OTPW),
            overtemperature_shutdown: bit(bits::OT),
            short_to_ground_a: bit(bits::S2GA),
            short_to_ground_b: bit(bits::S2GB),
            short_to_supply_a: bit(bits::S2VSA),
            short_to_supply_b: bit(bits::S2VSB),
            open_load_a: bit(bits::OLA),
            open_load_b: bit(bits::OLB),
            temperature_above_120c: bit(bits::T120),
            temperature_above_143c: bit(bits::T143),
            temperature_above_150c: bit(bits::T150),
            temperature_above_157c: bit(bits::T157),
            current_scale_actual: ((value >> bits::CS_ACTUAL_SHIFT) & bits::CS_ACTUAL_MASK) as u8,
            stealth_chop_active: bit(bits::STEALTH),
            standstill: bit(bits::STST),
        }
    }

    /// Возвращает `true`, если зафиксирована любая аварийная ситуация,
    /// требующая немедленного отключения драйвера (перегрев или короткое
    /// замыкание). Предупреждения (`overtemperature_prewarning`, обрыв
    /// обмотки) в эту категорию не входят.
    #[must_use]
    pub fn has_critical_fault(&self) -> bool {
        self.overtemperature_shutdown
            || self.short_to_ground_a
            || self.short_to_ground_b
            || self.short_to_supply_a
            || self.short_to_supply_b
    }
}

impl fmt::Display for DriverStatus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "DRV_STATUS {{ CS={}, {}{}{}{}{}{}{}{} }}",
            self.current_scale_actual,
            if self.standstill { "STST " } else { "" },
            if self.stealth_chop_active { "STEALTH " } else { "" },
            if self.overtemperature_prewarning { "OTPW " } else { "" },
            if self.overtemperature_shutdown { "OT! " } else { "" },
            if self.short_to_ground_a { "S2GA! " } else { "" },
            if self.short_to_ground_b { "S2GB! " } else { "" },
            if self.open_load_a { "OLA " } else { "" },
            if self.open_load_b { "OLB " } else { "" },
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_standstill_and_current_scale() {
        // STST=1 (бит 31), CS_ACTUAL=21 (0b10101, биты 16..=20).
        let raw = (1u32 << 31) | (0b10101 << 16);
        let status = DriverStatus::from_u32(raw);
        assert!(status.standstill);
        assert_eq!(status.current_scale_actual, 21);
        assert!(!status.has_critical_fault());
    }

    #[test]
    fn detects_critical_short_to_ground_fault() {
        let raw = 1u32 << 2; // S2GA
        let status = DriverStatus::from_u32(raw);
        assert!(status.short_to_ground_a);
        assert!(status.has_critical_fault());
    }
}
