//! Сетевая подсистема: Wi-Fi ([`wifi`]), HTTP API ([`http`]), телеметрия
//! WebSocket ([`websocket`]) и OTA-обновления (этот файл).
//!
//! `dead_code` временно отключён: модуль полностью реализован, но ещё не
//! создаётся `App` — потребуется системный цикл событий (`EspSystemEventLoop`)
//! и раздел NVS, которые появятся при финальной сборке прошивки.
#![allow(dead_code)]

pub mod http;
pub mod websocket;
pub mod wifi;

use crate::config::network::OtaConfig;
use crate::error::{AppError, AppResult};
use esp_idf_svc::ota::EspOta;
use std::io::Write;

/// Версия прошивки, известная на этапе компиляции (совпадает с версией,
/// сообщаемой `http::VersionInfo` и `M115`).
const FIRMWARE_VERSION: &str = env!("CARGO_PKG_VERSION");

/// Обновление прошивки по сети (`M115`-совместимая информация о текущей
/// прошивке доступна через [`OtaUpdater::running_firmware_info`]).
///
/// Приём самого образа (HTTP POST с двоичными данными) подключается на
/// уровне `network::http` при финальной сборке `App` — этот модуль
/// отвечает только за корректное применение уже полученных байт к
/// разделу OTA (`ota_0`/`ota_1`, см. `partitions.csv`).
pub struct OtaUpdater {
    config: OtaConfig,
}

/// Активная сессия записи образа обновления.
///
/// Пока сессия не завершена вызовом [`OtaSession::finish`], новый образ не
/// становится загрузочным — при потере питания посреди передачи станок
/// продолжит грузиться со старой, заведомо рабочей прошивки.
pub struct OtaSession {
    update: esp_idf_svc::ota::EspOtaUpdate<'static>,
    bytes_written: usize,
}

impl OtaUpdater {
    /// Создаёт обновлятель прошивки из конфигурации (`network.toml`,
    /// раздел `ota`).
    #[must_use]
    pub fn new(config: OtaConfig) -> Self {
        Self { config }
    }

    /// Возвращает `true`, если OTA-обновления разрешены конфигурацией.
    #[must_use]
    pub fn is_enabled(&self) -> bool {
        self.config.enabled
    }

    /// Человекочитаемое описание текущей запущенной прошивки: метка
    /// активного раздела OTA (`ota_0`/`ota_1`, см. `partitions.csv`) и
    /// версия, известная на этапе компиляции.
    ///
    /// Примечание для проверки при первой сборке: предполагается, что
    /// `esp_idf_svc::ota::Slot` содержит публичное поле `label: String` с
    /// именем раздела — эта часть API `esp-idf-svc` менее стабильна между
    /// версиями, чем `EspOta::initiate_update`/`EspOtaUpdate`, используемые
    /// в [`OtaUpdater::begin_update`] и не подвергающиеся вопросу.
    pub fn running_firmware_info(&self) -> AppResult<String> {
        let ota = EspOta::new().map_err(|e| AppError::Network(format!("не удалось получить доступ к OTA: {e}")))?;
        let slot = ota
            .get_running_slot()
            .map_err(|e| AppError::Network(format!("не удалось определить активный раздел OTA: {e}")))?;

        Ok(format!("{FIRMWARE_VERSION} (раздел {})", slot.label))
    }

    /// Начинает приём нового образа прошивки в неактивный OTA-раздел.
    ///
    /// Возвращает ошибку, если OTA отключён в конфигурации, либо если
    /// `require_signed_image` включён — проверка подписи образа
    /// (Secure Boot / signed app) не реализована на этом этапе, и молчаливо
    /// игнорировать связанную с безопасностью настройку недопустимо:
    /// лучше явно отказать в обновлении, чем притвориться, что проверка
    /// подписи выполняется.
    pub fn begin_update(&self) -> AppResult<OtaSession> {
        if !self.config.enabled {
            return Err(AppError::Network("OTA-обновления отключены в конфигурации".to_string()));
        }
        if self.config.require_signed_image {
            return Err(AppError::Network(
                "конфигурация требует проверки подписи образа (require_signed_image), но эта проверка ещё не реализована — обновление отклонено из соображений безопасности".to_string(),
            ));
        }

        let mut ota = EspOta::new().map_err(|e| AppError::Network(format!("не удалось получить доступ к OTA: {e}")))?;
        let update = ota
            .initiate_update()
            .map_err(|e| AppError::Network(format!("не удалось начать OTA-обновление: {e}")))?;

        log::info!("начато OTA-обновление прошивки");
        Ok(OtaSession { update, bytes_written: 0 })
    }
}

impl OtaSession {
    /// Записывает очередной фрагмент образа прошивки.
    pub fn write_chunk(&mut self, data: &[u8]) -> AppResult<()> {
        self.update
            .write_all(data)
            .map_err(|e| AppError::Network(format!("ошибка записи образа OTA: {e}")))?;
        self.bytes_written += data.len();
        Ok(())
    }

    /// Количество байт образа, записанных на данный момент.
    #[must_use]
    pub fn bytes_written(&self) -> usize {
        self.bytes_written
    }

    /// Завершает обновление: проверяет целостность принятого образа и
    /// помечает соответствующий раздел загрузочным. Станок должен быть
    /// перезагружен вызывающим кодом после успешного завершения, чтобы
    /// новая прошивка вступила в силу.
    pub fn finish(mut self) -> AppResult<()> {
        self.update
            .complete()
            .map_err(|e| AppError::Network(format!("не удалось завершить OTA-обновление: {e}")))?;
        log::info!("OTA-обновление завершено успешно ({} байт)", self.bytes_written);
        Ok(())
    }

    /// Прерывает обновление, не затрагивая текущую загрузочную прошивку.
    pub fn abort(mut self) -> AppResult<()> {
        self.update
            .abort()
            .map_err(|e| AppError::Network(format!("не удалось прервать OTA-обновление: {e}")))?;
        log::warn!("OTA-обновление прервано после {} байт", self.bytes_written);
        Ok(())
    }
}
