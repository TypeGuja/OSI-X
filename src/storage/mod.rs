//! Персистентное хранение настроек станка (`M500`/`M501`).
//!
//! [`SettingsManager`] — единственная точка входа для сохранения/загрузки
//! конфигурации: переиспользует TOML-сериализацию из [`crate::config`]
//! (`AppConfig::from_toml_parts`, `config::to_toml_string`, добавленные ещё
//! на этапе 1) поверх низкоуровневого доступа к разделу SPIFFS ([`settings::SettingsPartition`]).
//!
//! `dead_code` временно отключён: модуль полностью реализован, но ещё не
//! вызывается из `App` — конкретная реализация
//! `PrinterContext::save_settings`/`load_settings` появится при финальной
//! сборке прошивки.

pub mod settings;

use crate::config::AppConfig;
use crate::error::AppResult;
use settings::SettingsPartition;

/// Имена файлов конфигурации на разделе `settings`, в порядке,
/// соответствующем `config::AppConfig`.
const PRINTER_FILE: &str = "printer.toml";
const MOTION_FILE: &str = "motion.toml";
const NETWORK_FILE: &str = "network.toml";
const TEMPERATURE_FILE: &str = "temperature.toml";

/// Загружает и сохраняет [`AppConfig`] целиком на разделе `settings`.
pub struct SettingsManager {
    partition: SettingsPartition,
}

impl SettingsManager {
    /// Монтирует раздел `settings` и создаёт менеджер поверх него.
    pub fn mount() -> AppResult<Self> {
        Ok(Self {
            partition: SettingsPartition::mount()?,
        })
    }

    /// Создаёт менеджер поверх уже смонтированного раздела (используется,
    /// когда монтирование выполняется отдельно, например для повторного
    /// использования раздела другими подсистемами хранения).
    #[must_use]
    pub fn new(partition: SettingsPartition) -> Self {
        Self { partition }
    }

    /// Сохраняет полную конфигурацию (`M500`): каждый раздел записывается
    /// в отдельный файл, атомарно (см. [`SettingsPartition::write`]).
    pub fn save(&self, config: &AppConfig) -> AppResult<()> {
        let printer_toml = crate::config::to_toml_string(PRINTER_FILE, &config.printer)?;
        let motion_toml = crate::config::to_toml_string(MOTION_FILE, &config.motion)?;
        let network_toml = crate::config::to_toml_string(NETWORK_FILE, &config.network)?;
        let temperature_toml = crate::config::to_toml_string(TEMPERATURE_FILE, &config.temperature)?;

        self.partition.write(PRINTER_FILE, &printer_toml)?;
        self.partition.write(MOTION_FILE, &motion_toml)?;
        self.partition.write(NETWORK_FILE, &network_toml)?;
        self.partition.write(TEMPERATURE_FILE, &temperature_toml)?;

        log::info!("настройки сохранены на раздел 'settings'");
        Ok(())
    }

    /// Загружает конфигурацию (`M501`). Отсутствующие файлы (станок ни
    /// разу не сохранял настройки) трактуются как значения по умолчанию
    /// для соответствующего раздела — не ошибка.
    pub fn load(&self) -> AppResult<AppConfig> {
        let printer_toml = self.partition.read(PRINTER_FILE)?;
        let motion_toml = self.partition.read(MOTION_FILE)?;
        let network_toml = self.partition.read(NETWORK_FILE)?;
        let temperature_toml = self.partition.read(TEMPERATURE_FILE)?;

        let config = AppConfig::from_toml_parts(
            printer_toml.as_deref(),
            motion_toml.as_deref(),
            network_toml.as_deref(),
            temperature_toml.as_deref(),
        )?;

        log::info!("настройки загружены с раздела 'settings'");
        Ok(config)
    }

    /// Удаляет все сохранённые файлы конфигурации, возвращая станок к
    /// значениям по умолчанию при следующей загрузке.
    pub fn reset_to_defaults(&self) -> AppResult<()> {
        self.partition.delete(PRINTER_FILE)?;
        self.partition.delete(MOTION_FILE)?;
        self.partition.delete(NETWORK_FILE)?;
        self.partition.delete(TEMPERATURE_FILE)?;
        Ok(())
    }
}
