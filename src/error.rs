//! Единый тип ошибок для всей прошивки OSIX.
//!
//! Все модули возвращают [`AppError`] (напрямую или через `?` из своих
//! собственных ошибок, реализующих `From`), что позволяет `app.rs`
//! обрабатывать ошибки единообразно, не завязываясь на детали конкретного
//! драйвера или подсистемы.

use std::fmt;

/// Общий тип результата, используемый во всей прошивке.
pub type AppResult<T> = Result<T, AppError>;

/// Общая ошибка прошивки OSIX.
///
/// Каждый вариант соответствует конкретной подсистеме, что позволяет
/// вызывающему коду принимать решения (например, безопасно остановить
/// нагрев при [`AppError::Temperature`], не трогая моторы).
#[derive(Debug, thiserror::Error)]
pub enum AppError {
    /// Ошибка инициализации или работы платы (GPIO, питание, watchdog, RGB).
    #[error("ошибка платы: {0}")]
    Board(String),

    /// Ошибка низкоуровневого драйвера ESP-IDF (обёртка над `EspError`).
    #[error("ошибка ESP-IDF (код {code}): {message}")]
    Esp {
        /// Числовой код ошибки IDF (`esp_err_t`).
        code: i32,
        /// Человекочитаемое описание.
        message: String,
    },

    /// Ошибка драйвера двигателя (TMC2209, ULN2003 и т.д.).
    #[error("ошибка драйвера двигателя '{driver}': {reason}")]
    MotorDriver {
        /// Имя драйвера, в котором произошла ошибка.
        driver: &'static str,
        /// Причина ошибки.
        reason: String,
    },

    /// Ошибка планировщика движения (переполнение очереди, недопустимые
    /// параметры траектории и т.п.).
    #[error("ошибка планировщика движения: {0}")]
    Motion(String),

    /// Ошибка разбора или выполнения G-Code.
    #[error("ошибка G-Code (строка {line}): {reason}")]
    GCode {
        /// Номер строки в источнике команд.
        line: u32,
        /// Причина ошибки.
        reason: String,
    },

    /// Ошибка подсистемы температуры (термистор, thermal runaway и т.д.).
    #[error("ошибка температуры: {0}")]
    Temperature(String),

    /// Ошибка работы с картой памяти / файловой системой.
    #[error("ошибка SD-карты: {0}")]
    SdCard(String),

    /// Ошибка сети (Wi-Fi, HTTP, WebSocket).
    #[error("ошибка сети: {0}")]
    Network(String),

    /// Ошибка чтения/записи/парсинга конфигурации.
    #[error("ошибка конфигурации '{key}': {reason}")]
    Config {
        /// Ключ или файл конфигурации, вызвавший ошибку.
        key: String,
        /// Причина ошибки.
        reason: String,
    },

    /// Ошибка ввода-вывода общего назначения.
    #[error("ошибка ввода-вывода: {0}")]
    Io(String),

    /// Оборудование не отвечает или вернуло неожиданные данные.
    #[error("оборудование не отвечает: {0}")]
    HardwareTimeout(String),
}

impl AppError {
    /// Создаёт [`AppError::Board`] из любого сообщения.
    pub fn board(msg: impl Into<String>) -> Self {
        Self::Board(msg.into())
    }

    /// Создаёт [`AppError::MotorDriver`] с указанием имени драйвера.
    pub fn motor_driver(driver: &'static str, reason: impl Into<String>) -> Self {
        Self::MotorDriver {
            driver,
            reason: reason.into(),
        }
    }

    /// Создаёт [`AppError::Config`] с указанием ключа/файла конфигурации.
    pub fn config(key: impl Into<String>, reason: impl Into<String>) -> Self {
        Self::Config {
            key: key.into(),
            reason: reason.into(),
        }
    }
}

impl From<esp_idf_sys::EspError> for AppError {
    fn from(err: esp_idf_sys::EspError) -> Self {
        Self::Esp {
            code: err.code(),
            message: err.to_string(),
        }
    }
}

impl From<std::io::Error> for AppError {
    fn from(err: std::io::Error) -> Self {
        Self::Io(err.to_string())
    }
}

impl From<toml::de::Error> for AppError {
    fn from(err: toml::de::Error) -> Self {
        Self::Config {
            key: "<toml>".to_string(),
            reason: err.to_string(),
        }
    }
}

impl From<toml::ser::Error> for AppError {
    fn from(err: toml::ser::Error) -> Self {
        Self::Config {
            key: "<toml>".to_string(),
            reason: err.to_string(),
        }
    }
}

/// Вспомогательный трейт для приведения ошибок в контекст конкретного модуля.
///
/// Пример:
/// ```ignore
/// some_call().context_board("не удалось включить питание")?;
/// ```
pub trait ResultExt<T> {
    /// Оборачивает ошибку в [`AppError::Board`] с дополнительным контекстом.
    fn context_board(self, context: &str) -> AppResult<T>;
}

impl<T, E: fmt::Display> ResultExt<T> for Result<T, E> {
    fn context_board(self, context: &str) -> AppResult<T> {
        self.map_err(|e| AppError::Board(format!("{context}: {e}")))
    }
}
