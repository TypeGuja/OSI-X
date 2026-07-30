//! Контур нагревателя: термистор → ПИД → ШИМ, с защитой от
//! неконтролируемого нагрева (thermal runaway) и аварийного перегрева.

use crate::config::temperature::HeaterConfig;
use crate::error::{AppError, AppResult};
use crate::temperature::pid::PidController;
use crate::temperature::thermistor::{AnalogSample, Thermistor};

/// Источник ШИМ-сигнала нагревателя.
///
/// Обобщён отдельным трейтом (а не `esp_idf_hal::ledc`), чтобы логика
/// регулирования и защиты была тестируема на хосте — по аналогии с
/// [`crate::motion::step_generator::StepClock`] и
/// [`crate::drivers::motor::driver::MotorDriver`].
pub trait PwmOutput: Send {
    /// Устанавливает скважность в диапазоне `0..=255`.
    fn set_duty(&mut self, duty_0_255: u8) -> AppResult<()>;
}

/// Причина аварийной остановки нагревателя. Восстановление требует явного
/// вызова [`Heater::clear_fault`] (например, после подтверждения
/// пользователем) — контур никогда не возобновляет нагрев автоматически.
#[derive(Debug, Clone, PartialEq)]
pub enum HeaterFault {
    /// Измеренная температура превысила аварийный предел
    /// (`temperature.toml`, `max_temperature_c`).
    OverTemperature { measured_c: f32, limit_c: f32 },
    /// Температура не растёт при полной мощности нагрева достаточно быстро
    /// (обрыв нагревателя, отвалившийся термистор, короткое замыкание).
    NotHeating { elapsed_s: f32, rise_c: f32, required_c: f32 },
    /// Установившаяся температура отклонилась от целевой сильнее
    /// допустимого (застрявший в открытом состоянии силовой ключ,
    /// повреждённый термистор).
    ThermalRunaway { deviation_c: f32, limit_c: f32 },
    /// Ошибка чтения датчика температуры (обрыв/короткое замыкание).
    SensorFault(String),
}

impl std::fmt::Display for HeaterFault {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::OverTemperature { measured_c, limit_c } => {
                write!(f, "перегрев: {measured_c:.1}°C превышает предел {limit_c:.1}°C")
            }
            Self::NotHeating { elapsed_s, rise_c, required_c } => write!(
                f,
                "нагреватель не греется: за {elapsed_s:.1}с рост составил {rise_c:.1}°C (требовалось не менее {required_c:.1}°C)"
            ),
            Self::ThermalRunaway { deviation_c, limit_c } => write!(
                f,
                "неконтролируемый нагрев: отклонение {deviation_c:.1}°C превышает допуск {limit_c:.1}°C"
            ),
            Self::SensorFault(reason) => write!(f, "ошибка датчика температуры: {reason}"),
        }
    }
}

/// Внутреннее состояние наблюдателя thermal runaway (см.
/// [`crate::config::temperature::ThermalRunawayConfig`]).
enum RunawayWatch {
    /// Контур греется к цели: следим, что температура растёт достаточно
    /// быстро в течение окна наблюдения. Окно инициализируется лениво,
    /// при первом такте [`Heater::update`] после смены цели — на момент
    /// вызова [`Heater::set_target`] текущее время неизвестно.
    Heating { window: Option<(f64, f32)> },
    /// Контур на установившейся температуре: следим, что отклонение от
    /// цели не превышает допуск.
    Steady,
}

/// Контур одного нагревателя (хотэнд или стол): термистор + ПИД + ШИМ +
/// защита от thermal runaway/перегрева.
pub struct Heater<A: AnalogSample, P: PwmOutput> {
    thermistor: Thermistor<A>,
    pwm: P,
    pid: PidController,
    config: HeaterConfig,
    target_celsius: f32,
    last_measurement_c: f32,
    runaway_watch: RunawayWatch,
    fault: Option<HeaterFault>,
}

impl<A: AnalogSample, P: PwmOutput> Heater<A, P> {
    /// Создаёт контур нагревателя в выключенном состоянии (`target = 0`).
    pub fn new(thermistor: Thermistor<A>, pwm: P, config: HeaterConfig) -> Self {
        let pid = PidController::new(config.pid);
        Self {
            thermistor,
            pwm,
            pid,
            config,
            target_celsius: 0.0,
            last_measurement_c: 0.0,
            runaway_watch: RunawayWatch::Steady,
            fault: None,
        }
    }

    /// Устанавливает целевую температуру. `0.0` выключает нагрев.
    ///
    /// Отклоняется, если контур находится в состоянии аварии — вызывающий
    /// код должен сначала явно вызвать [`Heater::clear_fault`].
    pub fn set_target(&mut self, target_celsius: f32) -> AppResult<()> {
        if let Some(fault) = &self.fault {
            return Err(AppError::Temperature(format!(
                "невозможно установить целевую температуру: активна авария ({fault})"
            )));
        }
        self.target_celsius = target_celsius.max(0.0);
        self.pid.reset();
        self.runaway_watch = RunawayWatch::Heating { window: None };
        Ok(())
    }

    /// Текущая целевая температура.
    #[must_use]
    pub fn target_celsius(&self) -> f32 {
        self.target_celsius
    }

    /// Последнее измеренное значение температуры (может быть устаревшим,
    /// если [`Heater::update`] давно не вызывался).
    #[must_use]
    pub fn measured_celsius(&self) -> f32 {
        self.last_measurement_c
    }

    /// Возвращает `true`, если измеренная температура находится в пределах
    /// `tolerance_c` от целевой (используется блокирующим ожиданием
    /// `M109`/`M190`).
    #[must_use]
    pub fn is_at_target(&self, tolerance_c: f32) -> bool {
        (self.last_measurement_c - self.target_celsius).abs() <= tolerance_c
    }

    /// Текущая активная авария, если есть.
    #[must_use]
    pub fn fault(&self) -> Option<&HeaterFault> {
        self.fault.as_ref()
    }

    /// Сбрасывает зафиксированную аварию, немедленно выключая нагрев
    /// (`target = 0`) — контур не возобновляет нагрев автоматически.
    pub fn clear_fault(&mut self) {
        self.fault = None;
        self.target_celsius = 0.0;
        self.pid.reset();
    }

    /// Заменяет коэффициенты ПИД-регулятора (после автонастройки или
    /// загрузки `M501`).
    pub fn set_pid_gains(&mut self, config: crate::config::temperature::PidConfig) {
        self.pid.set_gains(config);
    }

    /// Один такт регулирования: читает температуру, проверяет защиту,
    /// пересчитывает ПИД и обновляет ШИМ.
    ///
    /// `time_s` — монотонное время в секундах от произвольного начала
    /// отсчёта (используется только для окон наблюдения thermal runaway).
    pub fn update(&mut self, dt_seconds: f32, time_s: f64) -> AppResult<()> {
        if self.fault.is_some() {
            self.pwm.set_duty(0)?;
            return Ok(());
        }

        let measurement = match self.thermistor.read_celsius() {
            Ok(value) => value,
            Err(AppError::Temperature(reason)) => {
                self.trigger_fault(HeaterFault::SensorFault(reason))?;
                return Ok(());
            }
            Err(other) => return Err(other),
        };
        self.last_measurement_c = measurement;

        if measurement >= self.config.max_temperature_c {
            self.trigger_fault(HeaterFault::OverTemperature {
                measured_c: measurement,
                limit_c: self.config.max_temperature_c,
            })?;
            return Ok(());
        }

        if self.config.thermal_runaway.enabled {
            if let Some(fault) = self.check_thermal_runaway(measurement, time_s) {
                self.trigger_fault(fault)?;
                return Ok(());
            }
        }

        let duty = if self.target_celsius > 0.0 {
            self.pid.update(self.target_celsius, measurement, dt_seconds)
        } else {
            self.pid.reset();
            0.0
        };

        self.pwm.set_duty(duty.round().clamp(0.0, 255.0) as u8)
    }

    /// Проверяет условия thermal runaway и возвращает аварию, если
    /// обнаружена. Не имеет побочных эффектов, кроме продвижения
    /// внутреннего окна наблюдения.
    fn check_thermal_runaway(&mut self, measurement_c: f32, time_s: f64) -> Option<HeaterFault> {
        let cfg = &self.config.thermal_runaway;
        let approaching_target = (measurement_c - self.target_celsius).abs() > cfg.hysteresis_c;

        match &mut self.runaway_watch {
            RunawayWatch::Heating { window } => {
                if !approaching_target {
                    self.runaway_watch = RunawayWatch::Steady;
                    return None;
                }

                let (window_start_s, temperature_at_window_start_c) = match *window {
                    Some(w) => w,
                    None => {
                        *window = Some((time_s, measurement_c));
                        return None;
                    }
                };

                let elapsed = (time_s - window_start_s) as f32;
                if elapsed >= cfg.period_s as f32 {
                    let rise = measurement_c - temperature_at_window_start_c;
                    if rise < cfg.hysteresis_c {
                        return Some(HeaterFault::NotHeating {
                            elapsed_s: elapsed,
                            rise_c: rise,
                            required_c: cfg.hysteresis_c,
                        });
                    }
                    *window = Some((time_s, measurement_c));
                }
                None
            }
            RunawayWatch::Steady => {
                if approaching_target && self.target_celsius > 0.0 {
                    // Цель сменилась достаточно сильно, чтобы снова следить
                    // за скоростью нагрева, а не за стабильностью.
                    self.runaway_watch = RunawayWatch::Heating {
                        window: Some((time_s, measurement_c)),
                    };
                    return None;
                }
                let deviation = (measurement_c - self.target_celsius).abs();
                if self.target_celsius > 0.0 && deviation > cfg.max_deviation_c {
                    return Some(HeaterFault::ThermalRunaway {
                        deviation_c: deviation,
                        limit_c: cfg.max_deviation_c,
                    });
                }
                None
            }
        }
    }

    /// Фиксирует аварию и немедленно обесточивает нагреватель.
    fn trigger_fault(&mut self, fault: HeaterFault) -> AppResult<()> {
        log::error!("авария нагревателя: {fault}");
        self.fault = Some(fault);
        self.target_celsius = 0.0;
        self.pwm.set_duty(0)
    }

    /// Один такт автонастройки ПИД методом реле ([`crate::temperature::pid::PidAutotune`]).
    ///
    /// В отличие от [`Heater::update`], не использует `self.pid` — мощность
    /// нагревателя напрямую задаётся релейным алгоритмом. Защита по
    /// максимальной температуре остаётся активной: превышение
    /// `max_temperature_c` немедленно прерывает автонастройку той же
    /// аварией, что и обычный режим работы.
    pub fn autotune_tick(
        &mut self,
        autotune: &mut crate::temperature::pid::PidAutotune,
        time_s: f64,
    ) -> AppResult<crate::temperature::pid::AutotuneStep> {
        use crate::temperature::pid::AutotuneStep;

        if let Some(fault) = &self.fault {
            self.pwm.set_duty(0)?;
            return Ok(AutotuneStep::Failed(format!("нагреватель в состоянии аварии: {fault}")));
        }

        let measurement = match self.thermistor.read_celsius() {
            Ok(value) => value,
            Err(AppError::Temperature(reason)) => {
                self.trigger_fault(HeaterFault::SensorFault(reason.clone()))?;
                return Ok(AutotuneStep::Failed(reason));
            }
            Err(other) => return Err(other),
        };
        self.last_measurement_c = measurement;

        if measurement >= self.config.max_temperature_c {
            let fault = HeaterFault::OverTemperature {
                measured_c: measurement,
                limit_c: self.config.max_temperature_c,
            };
            let message = fault.to_string();
            self.trigger_fault(fault)?;
            return Ok(AutotuneStep::Failed(message));
        }

        let step = autotune.sample(measurement, time_s);
        if let AutotuneStep::Continue { heater_power_fraction } = step {
            let duty = (heater_power_fraction.clamp(0.0, 1.0) * 255.0).round() as u8;
            self.pwm.set_duty(duty)?;
        } else {
            self.pwm.set_duty(0)?;
        }
        Ok(step)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::temperature::{PidConfig, ThermalRunawayConfig, ThermistorConfig, ThermistorModel};

    struct ConstantAdc {
        raw: u16,
    }
    impl AnalogSample for ConstantAdc {
        fn read_raw(&mut self) -> AppResult<u16> {
            Ok(self.raw)
        }
        fn max_value(&self) -> u16 {
            4095
        }
    }

    struct RecordingPwm {
        last_duty: u8,
    }
    impl PwmOutput for RecordingPwm {
        fn set_duty(&mut self, duty_0_255: u8) -> AppResult<()> {
            self.last_duty = duty_0_255;
            Ok(())
        }
    }

    fn make_heater(max_temperature_c: f32) -> Heater<ConstantAdc, RecordingPwm> {
        let thermistor = Thermistor::new(
            ConstantAdc { raw: 2048 },
            ThermistorConfig {
                model: ThermistorModel::Ntc100K3950,
                pullup_ohms: 100_000.0,
                oversampling: 1,
            },
        );
        let config = HeaterConfig {
            thermistor: ThermistorConfig {
                model: ThermistorModel::Ntc100K3950,
                pullup_ohms: 100_000.0,
                oversampling: 1,
            },
            pid: PidConfig { kp: 10.0, ki: 0.0, kd: 0.0, max_pwm: 255 },
            thermal_runaway: ThermalRunawayConfig {
                enabled: true,
                period_s: 10,
                hysteresis_c: 2.0,
                max_deviation_c: 10.0,
            },
            max_temperature_c,
        };
        Heater::new(thermistor, RecordingPwm { last_duty: 0 }, config)
    }

    #[test]
    fn heater_drives_pwm_toward_target_when_below() {
        let mut heater = make_heater(300.0);
        heater.set_target(200.0).unwrap();
        heater.update(1.0, 0.0).unwrap();
        assert!(heater.pwm.last_duty > 0, "нагреватель должен включиться ниже цели");
    }

    #[test]
    fn zero_target_keeps_heater_off() {
        let mut heater = make_heater(300.0);
        heater.update(1.0, 0.0).unwrap();
        assert_eq!(heater.pwm.last_duty, 0);
    }

    #[test]
    fn over_temperature_triggers_fault_and_shuts_off() {
        // Максимум установлен ниже температуры комнатной точки (~25°C),
        // чтобы гарантированно спровоцировать аварию на первом же такте.
        let mut heater = make_heater(20.0);
        heater.set_target(200.0).unwrap();
        heater.update(1.0, 0.0).unwrap();
        assert!(matches!(heater.fault(), Some(HeaterFault::OverTemperature { .. })));
        assert_eq!(heater.pwm.last_duty, 0);
    }

    #[test]
    fn fault_blocks_further_target_changes_until_cleared() {
        let mut heater = make_heater(20.0);
        heater.set_target(200.0).unwrap();
        heater.update(1.0, 0.0).unwrap();
        assert!(heater.set_target(50.0).is_err());
        heater.clear_fault();
        assert!(heater.set_target(50.0).is_ok());
    }

    #[test]
    fn not_heating_fault_triggers_when_temperature_does_not_rise() {
        let mut heater = make_heater(300.0);
        heater.set_target(200.0).unwrap();
        // Температура остаётся неизменной (термистор фиксирован на
        // комнатной точке) в течение всего окна наблюдения — должна
        // сработать защита "не греется".
        for i in 0..20 {
            heater.update(1.0, f64::from(i)).unwrap();
        }
        assert!(matches!(heater.fault(), Some(HeaterFault::NotHeating { .. })));
    }

    #[test]
    fn autotune_tick_respects_over_temperature_protection() {
        use crate::temperature::pid::PidAutotune;

        let mut heater = make_heater(20.0); // предел ниже комнатной точки
        let mut autotune = PidAutotune::new(200.0, 1.0, 5.0, 3);
        let step = heater.autotune_tick(&mut autotune, 0.0).unwrap();
        assert!(matches!(step, crate::temperature::pid::AutotuneStep::Failed(_)));
        assert!(matches!(heater.fault(), Some(HeaterFault::OverTemperature { .. })));
        assert_eq!(heater.pwm.last_duty, 0);
    }
}
