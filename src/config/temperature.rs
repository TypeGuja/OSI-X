//! Конфигурация подсистемы температуры: термисторы, нагреватели, PID,
//! thermal runaway (`temperature.toml`).

use serde::{Deserialize, Serialize};

/// Модель термистора (таблица сопротивление-температура).
///
/// На первом этапе поддерживаются стандартные таблицы NTC-термисторов,
/// используемых в 3D-печати; произвольная таблица может быть добавлена
/// позже без изменения структуры конфигурации.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ThermistorModel {
    /// NTC 100кОм при 25°C, бета 3950 (распространён на хотэндах).
    Ntc100K3950,
    /// NTC 100кОм при 25°C, бета 3435 (распространён на столах).
    Ntc100K3435,
}

impl Default for ThermistorModel {
    fn default() -> Self {
        Self::Ntc100K3950
    }
}

/// Конфигурация одного термистора.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ThermistorConfig {
    /// Модель термистора (таблица сопротивление-температура).
    pub model: ThermistorModel,
    /// Сопротивление подтягивающего резистора делителя напряжения, Ом.
    pub pullup_ohms: f32,
    /// Число АЦП-выборок, усредняемых для одного измерения (подавление шума).
    pub oversampling: u8,
}

impl Default for ThermistorConfig {
    fn default() -> Self {
        Self {
            model: ThermistorModel::default(),
            pullup_ohms: 4700.0,
            oversampling: 8,
        }
    }
}

/// Коэффициенты ПИД-регулятора нагревателя.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct PidConfig {
    /// Пропорциональный коэффициент.
    pub kp: f32,
    /// Интегральный коэффициент.
    pub ki: f32,
    /// Дифференциальный коэффициент.
    pub kd: f32,
    /// Максимальное значение ШИМ (0..=255), ограничивающее выход регулятора.
    pub max_pwm: u8,
}

impl Default for PidConfig {
    fn default() -> Self {
        Self {
            kp: 22.2,
            ki: 1.08,
            kd: 114.0,
            max_pwm: 255,
        }
    }
}

/// Параметры защиты от неконтролируемого нагрева (thermal runaway).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ThermalRunawayConfig {
    /// Включена ли защита.
    pub enabled: bool,
    /// Период наблюдения за ростом температуры, секунды.
    pub period_s: u32,
    /// Минимальный ожидаемый рост температуры за период при полной
    /// мощности нагрева, °C — если рост меньше, диагностируется
    /// неисправность нагревателя/термистора.
    pub hysteresis_c: f32,
    /// Максимально допустимое отклонение поддерживаемой температуры от
    /// целевой в установившемся режиме, °C.
    pub max_deviation_c: f32,
}

impl Default for ThermalRunawayConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            period_s: 20,
            hysteresis_c: 2.0,
            max_deviation_c: 10.0,
        }
    }
}

/// Полная конфигурация одного нагревательного контура (хотэнд или стол).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct HeaterConfig {
    /// Конфигурация термистора контура.
    pub thermistor: ThermistorConfig,
    /// Коэффициенты ПИД-регулятора.
    pub pid: PidConfig,
    /// Защита от thermal runaway.
    pub thermal_runaway: ThermalRunawayConfig,
    /// Максимально допустимая температура, °C — превышение приводит к
    /// немедленному аварийному отключению нагревателя.
    pub max_temperature_c: f32,
}

impl Default for HeaterConfig {
    fn default() -> Self {
        Self {
            thermistor: ThermistorConfig::default(),
            pid: PidConfig::default(),
            thermal_runaway: ThermalRunawayConfig::default(),
            max_temperature_c: 260.0,
        }
    }
}

/// Полная конфигурация подсистемы температуры (`temperature.toml`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct TemperatureConfig {
    /// Конфигурация хотэнда (экструдер 0).
    pub hotend: HeaterConfig,
    /// Конфигурация подогреваемого стола.
    pub bed: HeaterConfig,
    /// Период опроса термисторов, миллисекунды.
    pub sample_period_ms: u32,
}

impl Default for TemperatureConfig {
    fn default() -> Self {
        Self {
            hotend: HeaterConfig::default(),
            bed: HeaterConfig {
                pid: PidConfig {
                    kp: 120.0,
                    ki: 6.0,
                    kd: 400.0,
                    max_pwm: 255,
                },
                max_temperature_c: 120.0,
                ..HeaterConfig::default()
            },
            sample_period_ms: 250,
        }
    }
}
