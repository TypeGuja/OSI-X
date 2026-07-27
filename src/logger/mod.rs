//! Инициализация подсистемы логирования.
//!
//! Прошивка использует фасад `log` (`trace!`, `debug!`, `info!`, `warn!`,
//! `error!`) во всех модулях; фактический вывод осуществляется через
//! `EspLogger` из `esp-idf-svc`, который направляет сообщения в стандартный
//! вывод ESP-IDF (UART0/USB CDC) с префиксами тегов и уровней, совместимыми
//! с `idf.py monitor`.

use crate::error::AppResult;
use esp_idf_svc::log::EspLogger;
use log::LevelFilter;

/// Инициализирует глобальный логгер и устанавливает максимальный уровень
/// детализации.
///
/// Должна вызываться один раз, как можно раньше в `main`, до любых
/// вызовов `log::info!`/`log::warn!`/... из других модулей.
pub fn init(level: LevelFilter) -> AppResult<()> {
    EspLogger::initialize_default();
    log::set_max_level(level);
    log::info!("логирование инициализировано (уровень: {level})");
    Ok(())
}

/// Преобразует текстовое имя уровня логирования (например, из
/// конфигурации) в [`LevelFilter`]. Неизвестное значение трактуется как
/// `Info` — безопасный вариант по умолчанию для промышленной эксплуатации.
#[must_use]
pub fn level_from_str(name: &str) -> LevelFilter {
    match name.to_ascii_lowercase().as_str() {
        "trace" => LevelFilter::Trace,
        "debug" => LevelFilter::Debug,
        "warn" | "warning" => LevelFilter::Warn,
        "error" => LevelFilter::Error,
        "off" => LevelFilter::Off,
        _ => LevelFilter::Info,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unknown_level_defaults_to_info() {
        assert_eq!(level_from_str("banana"), LevelFilter::Info);
    }

    #[test]
    fn known_levels_are_parsed_case_insensitively() {
        assert_eq!(level_from_str("DEBUG"), LevelFilter::Debug);
        assert_eq!(level_from_str("Trace"), LevelFilter::Trace);
        assert_eq!(level_from_str("ERROR"), LevelFilter::Error);
    }
}
