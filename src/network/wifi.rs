//! Управление Wi-Fi: подключение к существующей сети (режим `Station`) или
//! создание собственной точки доступа для первичной настройки (режим
//! `AccessPoint`), согласно `network.toml` (см. `config::network::WifiConfig`).

use crate::config::network::WifiConfig;
use crate::error::{AppError, AppResult};
use esp_idf_hal::modem::Modem;
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::{AccessPointConfiguration, AuthMethod, ClientConfiguration, Configuration, EspWifi};
use std::net::Ipv4Addr;
use std::time::{Duration, Instant};

/// Максимальное время ожидания подключения к сети в режиме `Station`,
/// прежде чем попытка считается неудачной (одна из `max_connect_attempts`).
const CONNECT_TIMEOUT: Duration = Duration::from_secs(15);
/// Пароль по умолчанию для собственной точки доступа при первичной
/// настройке — намеренно не пустой (открытая сеть станка с ЧПУ была бы
/// небезопасна даже для временной настройки).
const ACCESS_POINT_DEFAULT_PASSWORD: &str = "osix-setup";

/// Информация о полученном сетевом адресе.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IpInfo {
    /// IPv4-адрес станка.
    pub ip: Ipv4Addr,
    /// Адрес шлюза.
    pub gateway: Ipv4Addr,
    /// Длина префикса подсети в нотации CIDR (например, `24` для `/24`).
    pub subnet_prefix_len: u8,
}

/// Управляет Wi-Fi-интерфейсом станка.
pub struct WifiManager<'d> {
    wifi: EspWifi<'d>,
}

impl<'d> WifiManager<'d> {
    /// Создаёт менеджер поверх модема, системного цикла событий и раздела
    /// NVS (используется драйвером Wi-Fi для хранения калибровочных
    /// данных радиотракта — не путать с настройками станка, которые
    /// хранятся на разделе `settings`, см. `storage`).
    pub fn new(
        modem: impl Peripheral<P = Modem> + 'd,
        sys_loop: EspSystemEventLoop,
        nvs: EspDefaultNvsPartition,
    ) -> AppResult<Self> {
        let wifi = EspWifi::new(modem, sys_loop, Some(nvs))
            .map_err(|e| AppError::Network(format!("не удалось инициализировать Wi-Fi: {e}")))?;
        Ok(Self { wifi })
    }

    /// Подключается к сети в режиме `Station`, используя `config`.
    /// Повторяет попытки до `config.max_connect_attempts` раз, ожидая до
    /// [`CONNECT_TIMEOUT`] на каждую. Возвращает ошибку, если ни одна
    /// попытка не увенчалась успехом — вызывающий код (финальная сборка
    /// `App`) в этом случае обычно переключается на [`WifiManager::start_access_point`].
    pub fn connect_station(&mut self, config: &WifiConfig) -> AppResult<IpInfo> {
        let ssid = to_heapless::<32>(&config.ssid, "SSID")?;
        let password = to_heapless::<64>(&config.password, "пароль Wi-Fi")?;

        let auth_method = if config.password.is_empty() {
            AuthMethod::None
        } else {
            AuthMethod::WPA2Personal
        };

        self.wifi
            .set_configuration(&Configuration::Client(ClientConfiguration {
                ssid,
                password,
                auth_method,
                ..Default::default()
            }))
            .map_err(|e| AppError::Network(format!("не удалось применить конфигурацию Wi-Fi: {e}")))?;

        self.wifi.start().map_err(|e| AppError::Network(format!("не удалось запустить Wi-Fi: {e}")))?;

        let attempts = config.max_connect_attempts.max(1);
        let mut last_error = None;

        for attempt in 1..=attempts {
            log::info!("подключение к Wi-Fi '{}', попытка {attempt}/{attempts}", config.ssid);
            match self.try_connect_once() {
                Ok(info) => return Ok(info),
                Err(e) => {
                    log::warn!("попытка подключения {attempt}/{attempts} не удалась: {e}");
                    last_error = Some(e);
                }
            }
        }

        Err(last_error.unwrap_or_else(|| AppError::Network("не удалось подключиться к Wi-Fi".to_string())))
    }

    /// Одна попытка подключения с ожиданием до [`CONNECT_TIMEOUT`].
    fn try_connect_once(&mut self) -> AppResult<IpInfo> {
        self.wifi.connect().map_err(|e| AppError::Network(format!("ошибка подключения: {e}")))?;

        let deadline = Instant::now() + CONNECT_TIMEOUT;
        while Instant::now() < deadline {
            let connected = self
                .wifi
                .is_connected()
                .map_err(|e| AppError::Network(format!("ошибка опроса состояния Wi-Fi: {e}")))?;
            if connected {
                return self.ip_info();
            }
            std::thread::sleep(Duration::from_millis(200));
        }

        Err(AppError::HardwareTimeout(
            "превышено время ожидания подключения к Wi-Fi".to_string(),
        ))
    }

    /// Создаёт собственную точку доступа с именем `ssid` (используется при
    /// первичной настройке станка, когда сохранённых учётных данных сети
    /// нет или подключение к ним не удалось).
    pub fn start_access_point(&mut self, ssid: &str) -> AppResult<()> {
        let ssid = to_heapless::<32>(ssid, "SSID точки доступа")?;
        let password = to_heapless::<64>(ACCESS_POINT_DEFAULT_PASSWORD, "пароль точки доступа")?;

        self.wifi
            .set_configuration(&Configuration::AccessPoint(AccessPointConfiguration {
                ssid,
                password,
                auth_method: AuthMethod::WPA2Personal,
                channel: 1,
                ..Default::default()
            }))
            .map_err(|e| AppError::Network(format!("не удалось применить конфигурацию точки доступа: {e}")))?;

        self.wifi
            .start()
            .map_err(|e| AppError::Network(format!("не удалось запустить точку доступа: {e}")))?;

        log::info!("точка доступа '{}' запущена (пароль по умолчанию — смените после настройки)", ACCESS_POINT_DEFAULT_PASSWORD);
        Ok(())
    }

    /// Возвращает `true`, если станок в данный момент подключён к сети.
    pub fn is_connected(&self) -> AppResult<bool> {
        self.wifi
            .is_connected()
            .map_err(|e| AppError::Network(format!("ошибка опроса состояния Wi-Fi: {e}")))
    }

    /// Текущая сетевая информация (действительна только при подключении).
    pub fn ip_info(&self) -> AppResult<IpInfo> {
        let info = self
            .wifi
            .sta_netif()
            .get_ip_info()
            .map_err(|e| AppError::Network(format!("не удалось получить сетевой адрес: {e}")))?;
        Ok(IpInfo {
            ip: info.ip,
            gateway: info.subnet.gateway,
            subnet_prefix_len: info.subnet.mask.0,
        })
    }
}

/// Переводит обычную строку в `heapless::String<N>` фиксированной ёмкости,
/// используемую полями конфигурации `esp-idf-svc` (`ssid`/`password`).
fn to_heapless<const N: usize>(value: &str, field_name: &str) -> AppResult<heapless::String<N>> {
    heapless::String::<N>::try_from(value)
        .map_err(|_| AppError::config("network.toml", format!("{field_name} длиннее {N} символов")))
}
