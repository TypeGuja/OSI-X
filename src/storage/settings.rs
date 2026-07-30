//! Монтирование раздела SPIFFS `settings` (см. `partitions.csv`) и
//! низкоуровневая работа с файлами на нём.
//!
//! Единственное место в прошивке, где вызывается C API SPIFFS
//! (`esp_vfs_spiffs_register`/`esp_vfs_spiffs_unregister`) — весь `unsafe`,
//! необходимый для монтирования раздела, изолирован в этом файле; после
//! монтирования файлы читаются и пишутся обычным безопасным `std::fs`,
//! поскольку ESP-IDF регистрирует SPIFFS как стандартную POSIX VFS.

use crate::error::{AppError, AppResult};
use esp_idf_sys::EspError;
use std::ffi::CString;
use std::fs;
use std::io::ErrorKind;
use std::path::PathBuf;

/// Метка раздела SPIFFS, зарезервированного под настройки (см. `partitions.csv`,
/// раздел `settings`, `0x40000` байт).
const PARTITION_LABEL: &str = "settings";
/// Точка монтирования в виртуальной файловой системе ESP-IDF.
const MOUNT_PATH: &str = "/settings";
/// Максимальное число одновременно открытых файлов на разделе — конфигурация
/// хранится в четырёх небольших TOML-файлах, читаемых/записываемых по
/// очереди, поэтому большого запаса не требуется.
const MAX_OPEN_FILES: usize = 4;

/// Смонтированный раздел настроек.
///
/// Монтирование выполняется один раз за время жизни программы (аналогично
/// [`crate::board::Board::init`], захватывающему `Peripherals` один раз) —
/// повторный вызов [`SettingsPartition::mount`] с тем же `partition_label`
/// без предварительного размонтирования вернёт ошибку ESP-IDF.
pub struct SettingsPartition {
    mount_path: &'static str,
}

impl SettingsPartition {
    /// Монтирует раздел `settings`, форматируя его при первом запуске или
    /// обнаружении повреждённой файловой системы (`format_if_mount_failed
    /// = true`) — для встраиваемого станка без консоли это единственный
    /// практичный вариант восстановления; потеря сохранённых настроек в
    /// этом случае предпочтительнее полностью неработоспособной прошивки.
    pub fn mount() -> AppResult<Self> {
        let base_path = CString::new(MOUNT_PATH).expect("путь монтирования не содержит NUL-байтов");
        let partition_label =
            CString::new(PARTITION_LABEL).expect("метка раздела не содержит NUL-байтов");

        let conf = esp_idf_sys::esp_vfs_spiffs_conf_t {
            base_path: base_path.as_ptr(),
            partition_label: partition_label.as_ptr(),
            max_files: MAX_OPEN_FILES,
            format_if_mount_failed: true,
        };

        // SAFETY: указатели в `conf` ссылаются на `CString`, живущие до
        // конца текущей функции — этого достаточно, поскольку
        // `esp_vfs_spiffs_register` копирует нужные данные внутри себя во
        // время вызова и не сохраняет переданные указатели после возврата.
        let ret = unsafe { esp_idf_sys::esp_vfs_spiffs_register(&conf) };
        EspError::convert(ret)
            .map_err(|e| AppError::board(format!("не удалось смонтировать раздел настроек: {e}")))?;

        log::info!("раздел настроек SPIFFS смонтирован в '{MOUNT_PATH}'");
        Ok(Self { mount_path: MOUNT_PATH })
    }

    /// Полный путь к файлу `name` на смонтированном разделе.
    fn file_path(&self, name: &str) -> PathBuf {
        PathBuf::from(self.mount_path).join(name)
    }

    /// Читает содержимое файла `name`, если он существует.
    ///
    /// Возвращает `Ok(None)`, если файла нет — это штатная ситуация при
    /// первом запуске станка, когда настройки ещё ни разу не сохранялись
    /// (`M500` ещё не вызывался), а не ошибка.
    pub fn read(&self, name: &str) -> AppResult<Option<String>> {
        match fs::read_to_string(self.file_path(name)) {
            Ok(contents) => Ok(Some(contents)),
            Err(e) if e.kind() == ErrorKind::NotFound => Ok(None),
            Err(e) => Err(AppError::from(e)),
        }
    }

    /// Записывает содержимое файла `name` атомарно: сначала во временный
    /// файл, затем переименовывает его поверх целевого. Это исключает
    /// повреждение настроек при потере питания посреди записи — после
    /// сбоя на диске остаётся либо старая версия файла, либо новая, но
    /// никогда не частично записанная.
    pub fn write(&self, name: &str, contents: &str) -> AppResult<()> {
        let path = self.file_path(name);
        let tmp_path = self.file_path(&format!("{name}.tmp"));

        fs::write(&tmp_path, contents)?;
        fs::rename(&tmp_path, &path)?;
        Ok(())
    }

    /// Удаляет файл `name`, если он существует (отсутствие файла не
    /// считается ошибкой — конечный результат совпадает с ожидаемым).
    pub fn delete(&self, name: &str) -> AppResult<()> {
        match fs::remove_file(self.file_path(name)) {
            Ok(()) => Ok(()),
            Err(e) if e.kind() == ErrorKind::NotFound => Ok(()),
            Err(e) => Err(AppError::from(e)),
        }
    }
}

impl Drop for SettingsPartition {
    fn drop(&mut self) {
        let Ok(partition_label) = CString::new(PARTITION_LABEL) else {
            return;
        };
        // SAFETY: размонтирование раздела, ранее успешно смонтированного в
        // `mount()`; ошибка на этапе уничтожения структуры не является
        // фатальной — прошивка либо перезагружается, либо завершает работу.
        let ret = unsafe { esp_idf_sys::esp_vfs_spiffs_unregister(partition_label.as_ptr()) };
        if let Err(e) = EspError::convert(ret) {
            log::warn!("не удалось размонтировать раздел настроек: {e}");
        }
    }
}
