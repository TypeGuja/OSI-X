//! Драйвер [`Tmc2209Driver`] — реализация [`MotorDriver`] поверх TMC2209
//! в режиме STEP/DIR с конфигурацией через однопроводный UART.

use super::registers::{address, ChopConf, CoolConf, GConf, GStat, IHoldIRun, MicrostepResolution, PwmConf};
use super::status::DriverStatus;
use super::uart::Tmc2209Uart;
use crate::drivers::motor::driver::MotorDriver;
use crate::error::{AppError, AppResult};
use crate::types::{Milliamps, MotorDirection};
use embedded_hal::digital::OutputPin;
use std::fmt::Debug;

/// Опорное напряжение полной шкалы регулятора тока при `VSENSE = 0`, вольт
/// (см. datasheet TMC2209, раздел "Current control").
const VFS_LOW_SENSITIVITY: f32 = 0.325;
/// Опорное напряжение полной шкалы при `VSENSE = 1` (повышенная
/// чувствительность, используется для малых токов).
const VFS_HIGH_SENSITIVITY: f32 = 0.180;

/// Параметры силовой части, необходимые для перевода миллиампер в шкалу
/// тока (`CS`, current scale) регистра `IHOLD_IRUN`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurrentSenseConfig {
    /// Сопротивление измерительного резистора драйвера, Ом (обычно `0.11`
    /// для типовых модулей TMC2209 на платах управления принтером).
    pub sense_resistor_ohms: f32,
}

impl Default for CurrentSenseConfig {
    fn default() -> Self {
        Self {
            sense_resistor_ohms: 0.11,
        }
    }
}

/// Драйвер двигателя оси на базе TMC2209 в режиме STEP/DIR.
///
/// Управление STEP/DIR/ENABLE выполняется через GPIO (обобщённые по
/// `embedded-hal`, что делает драйвер независимым от конкретного HAL), а
/// вся расширенная конфигурация (микрошаг, ток, StealthChop, SpreadCycle,
/// CoolStep, StallGuard) — через однопроводный UART ([`Tmc2209Uart`]).
pub struct Tmc2209Driver<'u, STEP, DIR, EN>
where
    STEP: OutputPin,
    DIR: OutputPin,
    EN: OutputPin,
{
    uart: Tmc2209Uart<'u>,
    step_pin: STEP,
    dir_pin: DIR,
    enable_pin: EN,
    current_sense: CurrentSenseConfig,
    chop_conf: ChopConf,
    enabled: bool,
}

impl<'u, STEP, DIR, EN> Tmc2209Driver<'u, STEP, DIR, EN>
where
    STEP: OutputPin,
    STEP::Error: Debug,
    DIR: OutputPin,
    DIR::Error: Debug,
    EN: OutputPin,
    EN::Error: Debug,
{
    /// Инициализирует драйвер: настраивает `GCONF` для управления по UART,
    /// сбрасывает `GSTAT`, применяет разумные значения по умолчанию для
    /// `CHOPCONF`, `PWMCONF`, `IHOLD_IRUN` и переводит выходы в выключенное
    /// состояние (`ENABLE` неактивен).
    ///
    /// `enable_pin` управляется активным низким уровнем — таково поведение
    /// вывода `ENN` на всех распространённых модулях TMC2209.
    pub fn init(
        uart: Tmc2209Uart<'u>,
        step_pin: STEP,
        dir_pin: DIR,
        mut enable_pin: EN,
        current_sense: CurrentSenseConfig,
    ) -> AppResult<Self> {
        enable_pin
            .set_high() // неактивно (выключено) при активном низком уровне
            .map_err(|e| AppError::motor_driver("tmc2209", format!("ENABLE pin: {e:?}")))?;

        let mut driver = Self {
            uart,
            step_pin,
            dir_pin,
            enable_pin,
            current_sense,
            chop_conf: ChopConf::default(),
            enabled: false,
        };

        driver.uart.write_register(address::GSTAT, GStat::clear_all())?;
        driver
            .uart
            .write_register(address::GCONF, GConf::default().to_u32())?;
        driver
            .uart
            .write_register(address::CHOPCONF, driver.chop_conf.to_u32())?;
        driver
            .uart
            .write_register(address::PWMCONF, PwmConf::default().to_u32())?;
        driver.uart.write_register(
            address::IHOLD_IRUN,
            IHoldIRun {
                ihold: 8,
                irun: 16,
                iholddelay: 4,
            }
            .to_u32(),
        )?;

        log::info!("TMC2209 инициализирован (UART, STEP/DIR, ENABLE активен низким уровнем)");
        Ok(driver)
    }

    /// Устанавливает разрешение микрошага (регистр `CHOPCONF.MRES`).
    pub fn set_microsteps(&mut self, resolution: MicrostepResolution) -> AppResult<()> {
        self.chop_conf.mres = resolution;
        self.uart
            .write_register(address::CHOPCONF, self.chop_conf.to_u32())?;
        log::debug!(
            "TMC2209: установлен микрошаг 1/{}",
            resolution.microsteps_per_step()
        );
        Ok(())
    }

    /// Включает или выключает интерполяцию микрошага до 256 шагов
    /// (сглаживание вращения независимо от фактического `MRES`).
    pub fn set_interpolation(&mut self, enabled: bool) -> AppResult<()> {
        self.chop_conf.intpol = enabled;
        self.uart
            .write_register(address::CHOPCONF, self.chop_conf.to_u32())
    }

    /// Переводит драйвер в режим StealthChop (тихий режим на основе
    /// voltage-PWM). Порог перехода на SpreadCycle на высоких скоростях
    /// задаётся [`Tmc2209Driver::set_stealth_chop_threshold`].
    pub fn enable_stealth_chop(&mut self) -> AppResult<()> {
        let mut gconf = GConf::default();
        gconf.en_spreadcycle = false;
        self.uart.write_register(address::GCONF, gconf.to_u32())?;
        log::debug!("TMC2209: включён StealthChop");
        Ok(())
    }

    /// Переводит драйвер в режим SpreadCycle (более высокий крутящий момент
    /// и эффективность на высоких скоростях за счёт акустического шума).
    pub fn enable_spread_cycle(&mut self) -> AppResult<()> {
        let mut gconf = GConf::default();
        gconf.en_spreadcycle = true;
        self.uart.write_register(address::GCONF, gconf.to_u32())?;
        log::debug!("TMC2209: включён SpreadCycle");
        Ok(())
    }

    /// Устанавливает порог `TPWMTHRS` — значение `TSTEP`, ниже которого
    /// драйвер автоматически переключается со StealthChop на SpreadCycle.
    /// `0` отключает автоматическое переключение (всегда StealthChop).
    pub fn set_stealth_chop_threshold(&mut self, tpwmthrs: u32) -> AppResult<()> {
        self.uart.write_register(address::TPWMTHRS, tpwmthrs)
    }

    /// Настраивает CoolStep — адаптивное снижение рабочего тока при низкой
    /// механической нагрузке на основе показаний StallGuard.
    pub fn set_coolstep(&mut self, config: CoolConf) -> AppResult<()> {
        self.uart.write_register(address::COOLCONF, config.to_u32())?;
        log::debug!("TMC2209: конфигурация CoolStep обновлена");
        Ok(())
    }

    /// Устанавливает порог скорости `TCOOLTHRS`, выше которого активны
    /// CoolStep и StallGuard.
    pub fn set_coolstep_threshold(&mut self, tcoolthrs: u32) -> AppResult<()> {
        self.uart.write_register(address::TCOOLTHRS, tcoolthrs)
    }

    /// Устанавливает порог срабатывания StallGuard (`SGTHRS`), `0..=255`.
    /// Меньшее значение — более чувствительное определение потери шага.
    pub fn set_stallguard_threshold(&mut self, sgthrs: u8) -> AppResult<()> {
        self.uart
            .write_register(address::SGTHRS, u32::from(sgthrs))
    }

    /// Читает текущее значение результата StallGuard (`SG_RESULT`),
    /// уменьшающееся при увеличении механической нагрузки на двигатель.
    pub fn read_stallguard_result(&mut self) -> AppResult<u16> {
        let raw = self.uart.read_register(address::SG_RESULT)?;
        Ok((raw & 0x03FF) as u16)
    }

    /// Читает диагностическое состояние драйвера (`DRV_STATUS`).
    pub fn read_status(&mut self) -> AppResult<DriverStatus> {
        let raw = self.uart.read_register(address::DRV_STATUS)?;
        Ok(DriverStatus::from_u32(raw))
    }

    /// Устанавливает рабочий ток и ток удержания в миллиамперах, вычисляя
    /// шкалу `CS` (`current scale`) по формуле datasheet:
    /// `I_rms = (CS + 1) / 32 * V_FS / (R_sense + 0.02) / sqrt(2)`.
    pub fn set_current(&mut self, run: Milliamps, hold: Milliamps, hold_delay: u8) -> AppResult<()> {
        let vfs = if self.chop_conf.vsense {
            VFS_HIGH_SENSITIVITY
        } else {
            VFS_LOW_SENSITIVITY
        };

        let irun = milliamps_to_current_scale(run, self.current_sense.sense_resistor_ohms, vfs);
        let ihold = milliamps_to_current_scale(hold, self.current_sense.sense_resistor_ohms, vfs);

        self.uart.write_register(
            address::IHOLD_IRUN,
            IHoldIRun {
                ihold,
                irun,
                iholddelay: hold_delay.min(IHoldIRun::MAX_HOLD_DELAY),
            }
            .to_u32(),
        )?;

        log::debug!(
            "TMC2209: установлен ток IRUN={run:?} (CS={irun}), IHOLD={hold:?} (CS={ihold})"
        );
        Ok(())
    }
}

/// Переводит целевой ток в миллиамперах (RMS) в шкалу `CS` (`0..=31`)
/// регистра `IHOLD_IRUN` по формуле из datasheet TMC2209.
fn milliamps_to_current_scale(target: Milliamps, sense_resistor_ohms: f32, vfs: f32) -> u8 {
    let target_a = f32::from(target.0) / 1000.0;
    let denominator = vfs / ((sense_resistor_ohms + 0.02) * std::f32::consts::SQRT_2);
    let cs = (target_a / denominator * 32.0) - 1.0;
    cs.round().clamp(0.0, f32::from(IHoldIRun::MAX_CURRENT_SCALE)) as u8
}

impl<'u, STEP, DIR, EN> MotorDriver for Tmc2209Driver<'u, STEP, DIR, EN>
where
    STEP: OutputPin,
    STEP::Error: Debug,
    DIR: OutputPin,
    DIR::Error: Debug,
    EN: OutputPin,
    EN::Error: Debug,
{
    fn enable(&mut self) -> AppResult<()> {
        self.enable_pin
            .set_low()
            .map_err(|e| AppError::motor_driver("tmc2209", format!("ENABLE pin: {e:?}")))?;
        self.enabled = true;
        Ok(())
    }

    fn disable(&mut self) -> AppResult<()> {
        self.enable_pin
            .set_high()
            .map_err(|e| AppError::motor_driver("tmc2209", format!("ENABLE pin: {e:?}")))?;
        self.enabled = false;
        Ok(())
    }

    fn is_enabled(&self) -> bool {
        self.enabled
    }

    fn set_direction(&mut self, direction: MotorDirection) -> AppResult<()> {
        match direction {
            MotorDirection::Forward => self.dir_pin.set_high(),
            MotorDirection::Backward => self.dir_pin.set_low(),
        }
        .map_err(|e| AppError::motor_driver("tmc2209", format!("DIR pin: {e:?}")))
    }

    fn step(&mut self) -> AppResult<()> {
        // Минимальная длительность высокого уровня STEP для TMC2209 — 100 нс,
        // что заведомо короче любой задержки, вносимой вызовом GPIO из Rust
        // поверх ESP-IDF, поэтому дополнительная программная задержка не
        // требуется.
        self.step_pin
            .set_high()
            .map_err(|e| AppError::motor_driver("tmc2209", format!("STEP pin: {e:?}")))?;
        self.step_pin
            .set_low()
            .map_err(|e| AppError::motor_driver("tmc2209", format!("STEP pin: {e:?}")))
    }

    fn set_speed(&mut self, _steps_per_second: f32) -> AppResult<()> {
        // Информационное значение: фактический тайминг шагов задаёт
        // `motion::step_generator`. TMC2209 сам определяет `TSTEP` по
        // физическим импульсам STEP и не требует программного указания
        // скорости для переключения StealthChop/SpreadCycle (см.
        // `set_stealth_chop_threshold`).
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
    fn current_scale_conversion_is_within_bounds() {
        let cs = milliamps_to_current_scale(Milliamps(800), 0.11, VFS_LOW_SENSITIVITY);
        assert!(cs <= IHoldIRun::MAX_CURRENT_SCALE);
    }

    #[test]
    fn zero_current_maps_to_minimum_scale() {
        let cs = milliamps_to_current_scale(Milliamps(0), 0.11, VFS_LOW_SENSITIVITY);
        assert_eq!(cs, 0);
    }

    #[test]
    fn excessive_current_saturates_at_maximum_scale() {
        let cs = milliamps_to_current_scale(Milliamps(5000), 0.11, VFS_LOW_SENSITIVITY);
        assert_eq!(cs, IHoldIRun::MAX_CURRENT_SCALE);
    }
}
