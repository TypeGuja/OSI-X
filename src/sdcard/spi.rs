//! Инициализация шины SPI для карты памяти.
//!
//! `esp_vfs_fat_sdspi_mount` (см. `fatfs.rs`) ожидает, что шина SPI уже
//! инициализирована вызывающим кодом через `spi_bus_initialize` — сам
//! SD-драйвер лишь регистрирует на этой шине устройство карты. Это
//! единственная функция в модуле `sdcard`, вызывающая `unsafe` напрямую;
//! остальной код (`fatfs.rs`, `mod.rs`) работает поверх уже
//! инициализированной шины.

use crate::board::pins::SdCardPins;
use crate::error::{AppError, AppResult};
use esp_idf_sys::EspError;
use std::mem::MaybeUninit;

/// Идентификатор SPI-хоста, используемого картой памяти (соответствует
/// периферии `SPI2`, зарезервированной в `Board::init`).
pub const SD_SPI_HOST: u32 = esp_idf_sys::spi_host_device_t_SPI2_HOST;

/// Максимальный размер одной SPI-транзакции, байт — определяет размер
/// DMA-буфера шины; `4000` с запасом покрывает секторы FAT (512 байт) и
/// характерные для FATFS размеры кластерных операций.
const MAX_TRANSFER_SIZE_BYTES: usize = 4000;

/// Инициализирует шину SPI на пинах карты памяти ([`SdCardPins`]) для
/// последующего использования SD-драйвером.
///
/// Должна вызываться ровно один раз для данного SPI-хоста за время жизни
/// программы — повторная инициализация уже занятой шины вернёт ошибку
/// ESP-IDF (`ESP_ERR_INVALID_STATE`).
pub fn init_bus(pins: &SdCardPins) -> AppResult<()> {
    // SAFETY: `spi_bus_config_t` — POD-структура; обнуление даёт корректные
    // значения по умолчанию для полей, которые мы не заполняем явно ниже
    // (флаги DMA-каналов данных 4..=7 октальной SPI, не используемых в
    // четырёхпроводном режиме SD-карты).
    let mut bus_config: esp_idf_sys::spi_bus_config_t = unsafe { MaybeUninit::zeroed().assume_init() };

    bus_config.__bindgen_anon_1.mosi_io_num = i32::from(pins.mosi);
    bus_config.__bindgen_anon_2.miso_io_num = i32::from(pins.miso);
    bus_config.sclk_io_num = i32::from(pins.sclk);
    bus_config.__bindgen_anon_3.quadwp_io_num = -1;
    bus_config.__bindgen_anon_4.quadhd_io_num = -1;
    bus_config.max_transfer_sz = MAX_TRANSFER_SIZE_BYTES as i32;
    bus_config.flags = 0;
    bus_config.intr_flags = 0;

    // SAFETY: `bus_config` заполнен корректными для текущего вызова
    // значениями и не сохраняется драйвером после возврата из
    // `spi_bus_initialize` (ESP-IDF копирует нужные поля во внутреннее
    // состояние шины). `SD_SPI_HOST` соответствует периферии `SPI2`,
    // зарезервированной и ещё не использованной на этот момент прошивки.
    //
    // ПРИМЕЧАНИЕ ДЛЯ ПРОВЕРКИ ПРИ ПЕРВОЙ СБОРКЕ: имена анонимных union-полей
    // (`__bindgen_anon_*`) генерируются `bindgen` и могут отличаться между
    // версиями `esp-idf-sys` — если сборка укажет на несовпадение имён,
    // сверьтесь с `esp_idf_sys::spi_bus_config_t`, сгенерированным для
    // конкретной версии ESP-IDF, указанной в `.cargo/config.toml`.
    let ret = unsafe {
        esp_idf_sys::spi_bus_initialize(SD_SPI_HOST, &bus_config, esp_idf_sys::spi_common_dma_t_SPI_DMA_CH_AUTO)
    };
    EspError::convert(ret).map_err(|e| AppError::board(format!("не удалось инициализировать шину SPI карты памяти: {e}")))?;

    log::info!("шина SPI карты памяти инициализирована (host {SD_SPI_HOST})");
    Ok(())
}
