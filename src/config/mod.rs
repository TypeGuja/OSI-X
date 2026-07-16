//! Конфигурация прошивки: `printer.toml`, `motion.toml`, `network.toml`,
//! `temperature.toml`.
//!
//! На этом этапе конфигурация существует как строго типизированные
//! структуры с разумными значениями по умолчанию и умеет
//! сериализоваться/десериализоваться в TOML. Персистентное хранение на
//! флеш-разделе `settings` (см. `partitions.csv`) подключается в модуле
//! `storage` на одном из следующих этапов — `AppConfig` уже сейчас не знает
//! о том, откуда взялась строка TOML, что позволяет добавить `storage` без
//! изменения текущего модуля.

pub mod motion;
pub mod network;
pub mod printer;
pub mod temperature;

use crate::error::{AppError, AppResult};
use motion::MotionConfig;
use network::NetworkConfig;
use printer::PrinterConfig;
use serde::{de::DeserializeOwned, Serialize};
use temperature::TemperatureConfig;

/// Полная конфигурация прошивки, объединяющая все отдельные разделы.
#[derive(Debug, Clone, PartialEq)]
pub struct AppConfig {
    /// Конфигурация принтера (`printer.toml`).
    pub printer: PrinterConfig,
    /// Конфигурация движения (`motion.toml`).
    pub motion: MotionConfig,
    /// Конфигурация сети (`network.toml`).
    pub network: NetworkConfig,
    /// Конфигурация температуры (`temperature.toml`).
    pub temperature: TemperatureConfig,
}

impl Default for AppConfig {
    fn default() -> Self {
        Self {
            printer: PrinterConfig::default(),
            motion: MotionConfig::default(),
            network: NetworkConfig::default(),
            temperature: TemperatureConfig::default(),
        }
    }
}

impl AppConfig {
    /// Собирает полную конфигурацию из четырёх отдельных TOML-строк.
    /// Отсутствующая или пустая строка означает "использовать значения по
    /// умолчанию" для соответствующего раздела.
    pub fn from_toml_parts(
        printer_toml: Option<&str>,
        motion_toml: Option<&str>,
        network_toml: Option<&str>,
        temperature_toml: Option<&str>,
    ) -> AppResult<Self> {
        Ok(Self {
            printer: parse_or_default("printer.toml", printer_toml)?,
            motion: parse_or_default("motion.toml", motion_toml)?,
            network: parse_or_default("network.toml", network_toml)?,
            temperature: parse_or_default("temperature.toml", temperature_toml)?,
        })
    }
}

/// Разбирает TOML-строку в тип `T`, либо возвращает значение по умолчанию,
/// если строка отсутствует или пуста.
fn parse_or_default<T>(source_name: &str, source: Option<&str>) -> AppResult<T>
where
    T: DeserializeOwned + Default,
{
    match source {
        Some(text) if !text.trim().is_empty() => toml::from_str(text)
            .map_err(|e| AppError::config(source_name, format!("ошибка разбора TOML: {e}"))),
        _ => Ok(T::default()),
    }
}

/// Сериализует значение конфигурации в TOML-строку, пригодную для
/// последующего сохранения через `storage::settings`.
pub fn to_toml_string<T: Serialize>(section_name: &str, value: &T) -> AppResult<String> {
    toml::to_string_pretty(value)
        .map_err(|e| AppError::config(section_name, format!("ошибка сериализации TOML: {e}")))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_config_roundtrips_through_toml() {
        let config = AppConfig::default();

        let printer_toml = to_toml_string("printer.toml", &config.printer).unwrap();
        let motion_toml = to_toml_string("motion.toml", &config.motion).unwrap();
        let network_toml = to_toml_string("network.toml", &config.network).unwrap();
        let temperature_toml = to_toml_string("temperature.toml", &config.temperature).unwrap();

        let restored = AppConfig::from_toml_parts(
            Some(&printer_toml),
            Some(&motion_toml),
            Some(&network_toml),
            Some(&temperature_toml),
        )
        .unwrap();

        assert_eq!(restored, config);
    }

    #[test]
    fn missing_sections_fall_back_to_defaults() {
        let restored = AppConfig::from_toml_parts(None, None, None, None).unwrap();
        assert_eq!(restored, AppConfig::default());
    }
}
