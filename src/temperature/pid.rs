//! ПИД-регулятор нагревателя ([`PidController`]) и автонастройка
//! коэффициентов методом реле Острёма-Хеглунда ([`PidAutotune`]),
//! используемая процедурой автонастройки PID.

use crate::config::temperature::PidConfig;

/// ПИД-регулятор с дифференцированием по измерению (а не по ошибке — во
/// избежание "выброса" при скачке уставки) и защитой от интегрального
/// насыщения (anti-windup): интегральная составляющая ограничивается так,
/// чтобы её вклад в выход не мог превысить `max_output` в одиночку.
pub struct PidController {
    kp: f32,
    ki: f32,
    kd: f32,
    max_output: f32,
    integral: f32,
    last_measurement: Option<f32>,
}

impl PidController {
    /// Создаёт регулятор из конфигурации (`temperature.toml`).
    #[must_use]
    pub fn new(config: PidConfig) -> Self {
        Self {
            kp: config.kp,
            ki: config.ki,
            kd: config.kd,
            max_output: f32::from(config.max_pwm),
            integral: 0.0,
            last_measurement: None,
        }
    }

    /// Пересчитывает выход регулятора по новому измерению.
    ///
    /// `dt_seconds` — время с предыдущего вызова [`PidController::update`];
    /// вызывающий код (`temperature::heater::Heater`) отвечает за
    /// поддержание постоянного периода опроса.
    ///
    /// Возвращает значение ШИМ в диапазоне `0.0..=max_pwm`.
    pub fn update(&mut self, setpoint_celsius: f32, measurement_celsius: f32, dt_seconds: f32) -> f32 {
        if dt_seconds <= 0.0 {
            return self.last_output(setpoint_celsius, measurement_celsius);
        }

        let error = setpoint_celsius - measurement_celsius;

        self.integral += error * dt_seconds;
        if self.ki.abs() > f32::EPSILON {
            let integral_limit = self.max_output / self.ki;
            self.integral = self.integral.clamp(-integral_limit.abs(), integral_limit.abs());
        } else {
            self.integral = 0.0;
        }

        let derivative = match self.last_measurement {
            Some(previous) => -(measurement_celsius - previous) / dt_seconds,
            None => 0.0,
        };
        self.last_measurement = Some(measurement_celsius);

        let output = self.kp * error + self.ki * self.integral + self.kd * derivative;
        output.clamp(0.0, self.max_output)
    }

    /// Возвращает пропорциональный вклад без обновления внутреннего
    /// состояния — используется только для деградированного поведения при
    /// нулевом/отрицательном `dt` (не должно происходить в штатной
    /// эксплуатации, но не должно и паниковать).
    fn last_output(&self, setpoint_celsius: f32, measurement_celsius: f32) -> f32 {
        (self.kp * (setpoint_celsius - measurement_celsius)).clamp(0.0, self.max_output)
    }

    /// Сбрасывает накопленную интегральную составляющую и историю
    /// дифференцирования — вызывается при смене уставки на существенно
    /// отличающееся значение, чтобы не наследовать интеграл от предыдущего
    /// режима работы.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.last_measurement = None;
    }

    /// Текущие коэффициенты регулятора в виде [`PidConfig`] (для `M500`).
    #[must_use]
    pub fn to_config(&self) -> PidConfig {
        PidConfig {
            kp: self.kp,
            ki: self.ki,
            kd: self.kd,
            max_pwm: self.max_output.round().clamp(0.0, 255.0) as u8,
        }
    }

    /// Заменяет коэффициенты регулятора (используется после автонастройки
    /// или при загрузке `M501`) и сбрасывает накопленное состояние.
    pub fn set_gains(&mut self, config: PidConfig) {
        self.kp = config.kp;
        self.ki = config.ki;
        self.kd = config.kd;
        self.max_output = f32::from(config.max_pwm);
        self.reset();
    }
}

/// Результат успешной автонастройки: рассчитанные коэффициенты ПИД и
/// промежуточные параметры релейного теста (для диагностики/логов).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutotuneResult {
    /// Рассчитанные коэффициенты ПИД (метод Зиглера-Николса по
    /// критическому усилению/периоду).
    pub pid: PidConfig,
    /// Критическое (предельное) усиление `Ku`, определённое из
    /// амплитуды колебаний.
    pub ultimate_gain: f32,
    /// Период колебаний `Tu`, секунды.
    pub ultimate_period_s: f32,
}

/// Шаг автонастройки — что делать с нагревателем прямо сейчас и не
/// завершился ли процесс.
#[derive(Debug, Clone, PartialEq)]
pub enum AutotuneStep {
    /// Продолжать: установить долю мощности нагревателя `heater_power_fraction`
    /// (`0.0` или `max_output_fraction` — релейный метод коммутирует между
    /// двумя уровнями, не использует промежуточные значения).
    Continue { heater_power_fraction: f32 },
    /// Автонастройка завершена успешно.
    Finished(AutotuneResult),
    /// Автонастройка не удалась (например, колебания не установились за
    /// разумное число циклов).
    Failed(String),
}

/// Автонастройка ПИД методом реле (Astrom-Hagglund): нагреватель
/// коммутируется между `0` и `max_output_fraction` при пересечении
/// уставки с гистерезисом, что вызывает устойчивые колебания температуры.
/// Из амплитуды и периода этих колебаний вычисляются критическое усиление
/// и период, а из них — коэффициенты ПИД по формулам Зиглера-Николса.
pub struct PidAutotune {
    target_celsius: f32,
    max_output_fraction: f32,
    hysteresis_c: f32,
    cycles_required: u8,
    completed_cycles: u8,
    heating: bool,
    peak_high_c: Option<f32>,
    peak_low_c: Option<f32>,
    last_switch_time_s: Option<f64>,
    half_periods_s: Vec<f32>,
    amplitudes_c: Vec<f32>,
}

impl PidAutotune {
    /// Начинает автонастройку с целевой температурой `target_celsius`.
    /// `cycles_required` — число полных циклов колебаний, по которым
    /// усредняются амплитуда и период (Marlin по умолчанию использует 5).
    #[must_use]
    pub fn new(target_celsius: f32, max_output_fraction: f32, hysteresis_c: f32, cycles_required: u8) -> Self {
        Self {
            target_celsius,
            max_output_fraction: max_output_fraction.clamp(0.0, 1.0),
            hysteresis_c: hysteresis_c.abs().max(0.1),
            cycles_required: cycles_required.max(1),
            completed_cycles: 0,
            heating: true,
            peak_high_c: None,
            peak_low_c: None,
            last_switch_time_s: None,
            half_periods_s: Vec::new(),
            amplitudes_c: Vec::new(),
        }
    }

    /// Обрабатывает новое измерение температуры в момент времени `time_s`
    /// (монотонные секунды от произвольного начала отсчёта).
    pub fn sample(&mut self, temperature_c: f32, time_s: f64) -> AutotuneStep {
        if self.heating {
            self.peak_high_c = Some(self.peak_high_c.map_or(temperature_c, |p| p.max(temperature_c)));

            if temperature_c >= self.target_celsius + self.hysteresis_c {
                self.heating = false;
                self.record_switch(time_s);
                self.peak_low_c = None;
                return AutotuneStep::Continue { heater_power_fraction: 0.0 };
            }
            AutotuneStep::Continue {
                heater_power_fraction: self.max_output_fraction,
            }
        } else {
            self.peak_low_c = Some(self.peak_low_c.map_or(temperature_c, |p| p.min(temperature_c)));

            if temperature_c <= self.target_celsius - self.hysteresis_c {
                self.heating = true;
                self.record_switch(time_s);

                if let (Some(high), Some(low)) = (self.peak_high_c, self.peak_low_c) {
                    self.amplitudes_c.push((high - low) / 2.0);
                }
                self.peak_high_c = None;
                self.completed_cycles += 1;

                if self.completed_cycles >= self.cycles_required {
                    return self.finish();
                }
                return AutotuneStep::Continue {
                    heater_power_fraction: self.max_output_fraction,
                };
            }
            AutotuneStep::Continue { heater_power_fraction: 0.0 }
        }
    }

    /// Фиксирует момент переключения реле, накапливая длительности
    /// полупериодов (интервалов между последовательными переключениями).
    fn record_switch(&mut self, time_s: f64) {
        if let Some(previous) = self.last_switch_time_s {
            self.half_periods_s.push((time_s - previous) as f32);
        }
        self.last_switch_time_s = Some(time_s);
    }

    /// Вычисляет итоговые коэффициенты ПИД из накопленных амплитуд и
    /// периодов. Первый полупериод отбрасывается как переходный процесс
    /// (температура ещё не вышла на установившиеся колебания).
    fn finish(&self) -> AutotuneStep {
        let half_periods = if self.half_periods_s.len() > 1 {
            &self.half_periods_s[1..]
        } else {
            &self.half_periods_s[..]
        };

        if half_periods.is_empty() || self.amplitudes_c.is_empty() {
            return AutotuneStep::Failed("недостаточно данных релейного теста для расчёта ПИД".to_string());
        }

        let mean_half_period = half_periods.iter().sum::<f32>() / half_periods.len() as f32;
        let ultimate_period_s = 2.0 * mean_half_period;

        let mean_amplitude = self.amplitudes_c.iter().sum::<f32>() / self.amplitudes_c.len() as f32;
        if mean_amplitude <= 0.01 {
            return AutotuneStep::Failed(
                "амплитуда колебаний слишком мала — проверьте нагреватель и датчик".to_string(),
            );
        }

        // Относительная амплитуда реле: коммутация 0 ↔ max_output_fraction
        // эквивалентна симметричному реле с амплитудой max_output_fraction/2
        // вокруг смещения max_output_fraction/2.
        let relay_amplitude = self.max_output_fraction / 2.0;
        let ultimate_gain = 4.0 * relay_amplitude / (std::f32::consts::PI * mean_amplitude);

        // Классические коэффициенты Зиглера-Николса для ПИД по Ku/Tu.
        let kp = 0.6 * ultimate_gain;
        let ki = 2.0 * kp / ultimate_period_s;
        let kd = kp * ultimate_period_s / 8.0;

        AutotuneStep::Finished(AutotuneResult {
            pid: PidConfig {
                kp,
                ki,
                kd,
                max_pwm: 255,
            },
            ultimate_gain,
            ultimate_period_s,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pid_output_is_proportional_to_error_on_first_update() {
        let mut pid = PidController::new(PidConfig { kp: 2.0, ki: 0.0, kd: 0.0, max_pwm: 255 });
        let output = pid.update(200.0, 190.0, 1.0);
        assert!((output - 20.0).abs() < 1e-3);
    }

    #[test]
    fn pid_output_is_clamped_to_max_pwm() {
        let mut pid = PidController::new(PidConfig { kp: 100.0, ki: 0.0, kd: 0.0, max_pwm: 255 });
        let output = pid.update(200.0, 0.0, 1.0);
        assert!((output - 255.0).abs() < 1e-3);
    }

    #[test]
    fn integral_term_accumulates_over_time() {
        let mut pid = PidController::new(PidConfig { kp: 0.0, ki: 1.0, kd: 0.0, max_pwm: 255 });
        let first = pid.update(100.0, 90.0, 1.0);
        let second = pid.update(100.0, 90.0, 1.0);
        assert!(second > first, "интеграл должен продолжать накапливаться при неизменной ошибке");
    }

    #[test]
    fn reset_clears_integral_and_derivative_history() {
        let mut pid = PidController::new(PidConfig { kp: 0.0, ki: 1.0, kd: 0.0, max_pwm: 255 });
        pid.update(100.0, 90.0, 1.0);
        pid.reset();
        let after_reset = pid.update(100.0, 90.0, 1.0);
        // После сброса интеграл начинает накопление заново — за один шаг
        // такой же длительности вклад должен совпасть с самым первым вызовом.
        assert!((after_reset - 10.0).abs() < 1e-3);
    }

    #[test]
    fn autotune_recovers_known_period_and_produces_positive_gains() {
        // Синтетический прямоугольный сигнал с известной амплитудой (5°C)
        // и периодом (20с) вокруг уставки 200°C, поданный напрямую как
        // измерения — эмулирует установившиеся колебания релейного теста.
        let mut autotune = PidAutotune::new(200.0, 1.0, 0.1, 3);

        let mut time = 0.0f64;
        let mut result = None;
        // Пилообразный сигнал: линейный рост/спад между 195 и 205°C.
        for _ in 0..2000 {
            let phase = (time % 20.0) as f32;
            let temperature = if phase < 10.0 {
                195.0 + phase // рост 195 -> 205
            } else {
                205.0 - (phase - 10.0) // спад 205 -> 195
            };
            match autotune.sample(temperature, time) {
                AutotuneStep::Finished(r) => {
                    result = Some(r);
                    break;
                }
                AutotuneStep::Failed(reason) => panic!("автонастройка не удалась: {reason}"),
                AutotuneStep::Continue { .. } => {}
            }
            time += 0.05;
        }

        let result = result.expect("автонастройка должна завершиться за отведённое число итераций");
        assert!(result.pid.kp > 0.0);
        assert!(result.pid.ki > 0.0);
        assert!(result.pid.kd > 0.0);
        // Период должен быть в разумных пределах около заданных 20с.
        assert!(result.ultimate_period_s > 10.0 && result.ultimate_period_s < 30.0, "период {}", result.ultimate_period_s);
    }
}
