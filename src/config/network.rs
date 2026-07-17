//! Конфигурация сетевой подсистемы: Wi-Fi, HTTP API, WebSocket, OTA
//! (`network.toml`).

use serde::{Deserialize, Serialize};

/// Режим работы Wi-Fi.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WifiMode {
    /// Станция — подключение к существующей точке доступа.
    Station,
    /// Точка доступа — станок сам создаёт сеть (используется при первичной
    /// настройке, если сохранённых учётных данных нет).
    AccessPoint,
}

impl Default for WifiMode {
    fn default() -> Self {
        Self::Station
    }
}

/// Учётные данные и режим Wi-Fi.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct WifiConfig {
    /// Режим работы.
    pub mode: WifiMode,
    /// SSID сети (в режиме `Station`) или создаваемой сети (в режиме
    /// `AccessPoint`).
    pub ssid: String,
    /// Пароль сети. Хранится в конфигурации, находящейся в разделе
    /// `settings` флеш-памяти; передача по сети (HTTP API) не предусмотрена.
    pub password: String,
    /// Пытаться ли переподключаться автоматически при разрыве соединения.
    pub auto_reconnect: bool,
    /// Максимальное число попыток подключения при старте, прежде чем
    /// перейти в режим `AccessPoint` для настройки.
    pub max_connect_attempts: u8,
}

impl Default for WifiConfig {
    fn default() -> Self {
        Self {
            mode: WifiMode::default(),
            ssid: String::new(),
            password: String::new(),
            auto_reconnect: true,
            max_connect_attempts: 5,
        }
    }
}

/// Конфигурация встроенного HTTP API.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct HttpApiConfig {
    /// Включён ли HTTP API.
    pub enabled: bool,
    /// TCP-порт HTTP-сервера.
    pub port: u16,
    /// Максимальное число одновременных соединений.
    pub max_connections: u8,
}

impl Default for HttpApiConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            port: 80,
            max_connections: 4,
        }
    }
}

/// Конфигурация WebSocket-канала для потоковой передачи статуса печати.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct WebSocketConfig {
    /// Включён ли WebSocket.
    pub enabled: bool,
    /// TCP-порт WebSocket-сервера.
    pub port: u16,
    /// Интервал отправки телеметрии (позиция, температуры), миллисекунды.
    pub telemetry_interval_ms: u32,
}

impl Default for WebSocketConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            port: 81,
            telemetry_interval_ms: 250,
        }
    }
}

/// Конфигурация OTA-обновлений.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct OtaConfig {
    /// Разрешены ли OTA-обновления по сети.
    pub enabled: bool,
    /// Требовать ли проверку подписи образа перед применением.
    pub require_signed_image: bool,
}

impl Default for OtaConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            require_signed_image: true,
        }
    }
}

/// Полная конфигурация сетевой подсистемы (`network.toml`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct NetworkConfig {
    /// Настройки Wi-Fi.
    pub wifi: WifiConfig,
    /// Настройки HTTP API.
    pub http: HttpApiConfig,
    /// Настройки WebSocket.
    pub websocket: WebSocketConfig,
    /// Настройки OTA.
    pub ota: OtaConfig,
    /// Имя хоста (mDNS), под которым станок виден в локальной сети.
    pub hostname: String,
}

impl Default for NetworkConfig {
    fn default() -> Self {
        Self {
            wifi: WifiConfig::default(),
            http: HttpApiConfig::default(),
            websocket: WebSocketConfig::default(),
            ota: OtaConfig::default(),
            hostname: "osix-printer".to_string(),
        }
    }
}
