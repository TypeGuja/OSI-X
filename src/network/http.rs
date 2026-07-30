//! Встроенный HTTP API сервер (`network.toml`, раздел `http`).
//!
//! Предоставляет базовый эндпоинт `/api/version` "из коробки" и точку
//! расширения ([`HttpApiServer::server_mut`]) для регистрации остальных
//! обработчиков (статус станка, список файлов SD-карты, приём G-Code,
//! приём образа OTA) — они требуют доступа к `PrinterContext`,
//! `sdcard::SdCard` и [`crate::network::OtaUpdater`] одновременно, поэтому
//! регистрируются при финальной сборке `App`, а не здесь.

use crate::config::network::HttpApiConfig;
use crate::error::{AppError, AppResult};
use esp_idf_svc::http::server::{Configuration as HttpServerConfiguration, EspHttpServer};
use esp_idf_svc::http::Method;
use serde::Serialize;
use std::io::Write;

/// Версия прошивки, сообщаемая `/api/version` (совпадает по духу с `M115`
/// в `gcode::commands::system`, но в машиночитаемом JSON-формате для
/// веб-интерфейсов).
#[derive(Debug, Clone, Serialize)]
struct VersionInfo {
    firmware: &'static str,
    version: &'static str,
    board: &'static str,
}

const FIRMWARE_NAME: &str = "OSIX Firmware";
const FIRMWARE_VERSION: &str = env!("CARGO_PKG_VERSION");
const BOARD_NAME: &str = "ESP32-S3 N16R8";

/// Встроенный HTTP API сервер станка.
pub struct HttpApiServer {
    server: EspHttpServer<'static>,
}

impl HttpApiServer {
    /// Запускает сервер на порту из `config`, регистрируя базовый
    /// эндпоинт `/api/version`.
    ///
    /// Возвращает `Ok(None)`, если сервер отключён в конфигурации
    /// (`config.enabled == false`) — вызывающий код в этом случае просто
    /// не хранит `HttpApiServer` и не тратит порт/ресурсы.
    pub fn start(config: &HttpApiConfig) -> AppResult<Option<Self>> {
        if !config.enabled {
            log::info!("HTTP API отключён в конфигурации");
            return Ok(None);
        }

        let server_config = HttpServerConfiguration {
            http_port: config.port,
            max_sessions: usize::from(config.max_connections),
            ..Default::default()
        };

        let mut server = EspHttpServer::new(&server_config)
            .map_err(|e| AppError::Network(format!("не удалось запустить HTTP-сервер: {e}")))?;

        server
            .fn_handler("/api/version", Method::Get, |request| -> anyhow::Result<()> {
                let payload = VersionInfo {
                    firmware: FIRMWARE_NAME,
                    version: FIRMWARE_VERSION,
                    board: BOARD_NAME,
                };
                let body = serde_json::to_vec(&payload)?;
                let mut response = request.into_ok_response()?;
                response.write_all(&body)?;
                Ok(())
            })
            .map_err(|e| AppError::Network(format!("не удалось зарегистрировать /api/version: {e}")))?;

        log::info!("HTTP API запущен на порту {}", config.port);
        Ok(Some(Self { server }))
    }

    /// Прямой доступ к серверу для регистрации дополнительных обработчиков
    /// на этапе финальной сборки `App` (статус станка, файлы SD-карты,
    /// приём G-Code, OTA).
    pub fn server_mut(&mut self) -> &mut EspHttpServer<'static> {
        &mut self.server
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn version_info_serializes_to_expected_json_shape() {
        let payload = VersionInfo {
            firmware: FIRMWARE_NAME,
            version: "0.1.0",
            board: BOARD_NAME,
        };
        let json = serde_json::to_string(&payload).unwrap();
        assert!(json.contains("\"firmware\":\"OSIX Firmware\""));
        assert!(json.contains("\"board\":\"ESP32-S3 N16R8\""));
    }
}
