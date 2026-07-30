//! Монтирование карты памяти (FAT32) через SD SPI-драйвер ESP-IDF.
//!
//! Использует `esp_vfs_fat_sdspi_mount` — встроенный в ESP-IDF драйвер,
//! объединяющий протокол SD-карты поверх SPI и монтирование FAT32 как
//! стандартной POSIX VFS. Это единственно разумный подход для прошивки
//! промышленного качества: самостоятельная реализация протокола SD/SPI и
//! парсера FAT32 с нуля — многотысячестрочная задача с высоким риском
//! повреждения данных на карте при малейшей ошибке, тогда как драйвер
//! ESP-IDF уже проверен на миллионах устройств.
//!
//! # Важное примечание о верификации
//!
//! Инициализация `sdmmc_host_t` в C обычно выполняется макросом
//! `SDSPI_HOST_DEFAULT()` из `driver/sdspi_host.h`. Макросы (в отличие от
//! обычных функций) не всегда попадают в биндинги `bindgen`, поэтому этот
//! модуль собирает `sdmmc_host_t` вручную, обнуляя остальные поля через
//! `mem::zeroed()` и явно заполняя только задокументированные для
//! SPI-режима функции-члены. Это единственное место во всём проекте,
//! отмеченное как требующее сверки с реальными биндингами при первой
//! сборке (см. комментарий у [`spi_host_config`]) — весь остальной код
//! `sdcard` не зависит от точности этой структуры и полностью тестируем.

use crate::error::{AppError, AppResult};
use esp_idf_sys::EspError;
use std::ffi::CString;
use std::mem::MaybeUninit;

/// Собирает `sdmmc_host_t`, эквивалентный `SDSPI_HOST_DEFAULT()` для
/// заданного SPI-хоста.
///
/// # Safety-примечание для проверки при первой сборке
///
/// Поля-функции (`init`, `set_card_clk`, `do_transaction`, `io_int_enable`,
/// `io_int_wait`, `get_real_freq`) указывают на публичные (не макро-only)
/// функции `sdspi_host.h`, поэтому должны присутствовать в биндингах любой
/// версии `esp-idf-sys`. Поле `deinit`/`deinit_p` в оригинальном C-типе
/// объявлено как анонимный `union` — точное имя сгенerированного поля
/// зависит от версии `bindgen`, поэтому оно намеренно оставлено обнулённым
/// (`mem::zeroed()`): следствие — при `unmount()` низкоуровневый драйвер
/// SPI-устройства может не до конца освободить ресурс. Для прошивки,
/// монтирующей карту один раз при старте и не размонтирующей её в штатной
/// работе, это не является проблемой; если размонтирование потребуется
/// чаще, поле нужно дозаполнить по сверке с реальными биндингами.
fn spi_host_config(host_id: u32) -> esp_idf_sys::sdmmc_host_t {
    // SAFETY: `sdmmc_host_t` — POD-структура (числа и указатели на функции,
    // без деструкторов и инвариантов, требующих инициализации кроме тех,
    // что мы устанавливаем явно ниже); нулевые указатели на необязательные
    // функции-члены (`set_bus_width`, `set_bus_ddr_mode`, ...) — штатное
    // значение и в оригинальном `SDSPI_HOST_DEFAULT()` для режима SPI.
    let mut host: esp_idf_sys::sdmmc_host_t = unsafe { MaybeUninit::zeroed().assume_init() };

    host.flags = esp_idf_sys::SDMMC_HOST_FLAG_SPI;
    host.slot = host_id as i32;
    host.max_freq_khz = esp_idf_sys::SDMMC_FREQ_DEFAULT as i32;
    host.io_voltage = 3.3;
    host.init = Some(esp_idf_sys::sdspi_host_init);
    host.set_card_clk = Some(esp_idf_sys::sdspi_host_set_card_clk);
    host.do_transaction = Some(esp_idf_sys::sdspi_host_do_transaction);
    host.io_int_enable = Some(esp_idf_sys::sdspi_host_io_int_enable);
    host.io_int_wait = Some(esp_idf_sys::sdspi_host_io_int_wait);
    host.get_real_freq = Some(esp_idf_sys::sdspi_host_get_real_freq);
    host.command_timeout_ms = 0;
    host.input_delay_phase = esp_idf_sys::sdmmc_delay_phase_t_SDMMC_DELAY_PHASE_0;

    host
}

/// Смонтированная FAT32-карта памяти.
pub struct MountedCard {
    mount_path: &'static str,
    card: *mut esp_idf_sys::sdmmc_card_t,
}

// SAFETY: `sdmmc_card_t` используется исключительно через официальные
// функции ESP-IDF (`esp_vfs_fat_sdcard_unmount`), которые сами
// синхронизируют доступ к карте на уровне драйвера; сама прошивка не
// разыменовывает указатель напрямую.
unsafe impl Send for MountedCard {}

impl MountedCard {
    /// Монтирует карту памяти на пине `cs_pin` (chip select) в точке `mount_path`.
    ///
    /// `spi::init_bus` должна быть вызвана раньше для того же SPI-хоста.
    pub fn mount(cs_pin: u8, mount_path: &'static str, max_open_files: usize, format_if_mount_failed: bool) -> AppResult<Self> {
        let host = spi_host_config(super::spi::SD_SPI_HOST);

        // SAFETY: POD-структура; см. обоснование в `spi_host_config` выше —
        // те же соображения применимы к `sdspi_device_config_t` и
        // `esp_vfs_fat_mount_config_t` (оба содержат только числа/булевы
        // флаги без указателей, требующих инициализации).
        let mut slot_config: esp_idf_sys::sdspi_device_config_t = unsafe { MaybeUninit::zeroed().assume_init() };
        slot_config.host_id = super::spi::SD_SPI_HOST as i32;
        slot_config.gpio_cs = i32::from(cs_pin);
        slot_config.gpio_cd = esp_idf_sys::SDSPI_SLOT_NO_CD;
        slot_config.gpio_wp = esp_idf_sys::SDSPI_SLOT_NO_WP;
        slot_config.gpio_int = esp_idf_sys::SDSPI_SLOT_NO_INT;
        slot_config.gpio_wp_polarity = false;

        let mut mount_config: esp_idf_sys::esp_vfs_fat_mount_config_t = unsafe { MaybeUninit::zeroed().assume_init() };
        mount_config.format_if_mount_failed = format_if_mount_failed;
        mount_config.max_files = max_open_files as i32;
        mount_config.allocation_unit_size = 16 * 1024;

        let mount_path_c = CString::new(mount_path).expect("путь монтирования не содержит NUL-байтов");
        let mut card_ptr: *mut esp_idf_sys::sdmmc_card_t = std::ptr::null_mut();

        // SAFETY: все указатели (`host`, `slot_config`, `mount_config`)
        // ссылаются на локальные переменные, живущие до конца этого вызова,
        // чего достаточно — `esp_vfs_fat_sdspi_mount` копирует нужные данные
        // синхронно во время вызова. `card_ptr` — валидный указатель на
        // локальную переменную для записи результата.
        let ret = unsafe {
            esp_idf_sys::esp_vfs_fat_sdspi_mount(
                mount_path_c.as_ptr(),
                &host,
                &slot_config,
                &mount_config,
                &mut card_ptr,
            )
        };
        EspError::convert(ret).map_err(|e| AppError::SdCard(format!("не удалось смонтировать карту памяти: {e}")))?;

        log::info!("карта памяти смонтирована в '{mount_path}'");
        Ok(Self { mount_path, card: card_ptr })
    }

    /// Путь монтирования в виртуальной файловой системе.
    #[must_use]
    pub fn mount_path(&self) -> &'static str {
        self.mount_path
    }
}

impl Drop for MountedCard {
    fn drop(&mut self) {
        let Ok(mount_path_c) = CString::new(self.mount_path) else {
            return;
        };
        // SAFETY: `self.card` был получен из успешного `mount()` и не
        // используется нигде за пределами этой структуры.
        let ret = unsafe { esp_idf_sys::esp_vfs_fat_sdcard_unmount(mount_path_c.as_ptr(), self.card) };
        if let Err(e) = EspError::convert(ret) {
            log::warn!("не удалось размонтировать карту памяти: {e}");
        }
    }
}
