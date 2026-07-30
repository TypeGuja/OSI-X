//! Карта памяти: SPI + FAT32, печать напрямую с карты.
//!
//! [`spi`] и [`fatfs`] отвечают за низкоуровневое монтирование (см. их
//! документацию по поводу единственного не полностью верифицированного
//! места во всём проекте). Всё в этом файле — обычный безопасный Rust
//! поверх `std::fs`, работающий после успешного монтирования, и полностью
//! покрыт хостовыми тестами на временных файлах.
//!
//! `dead_code` временно отключён: модуль полностью реализован, но ещё не
//! вызывается из `App` — потребуется подключение реальных пинов SD-карты
//! из `Board` при финальной сборке прошивки.
#![allow(dead_code)]

pub mod fatfs;
pub mod spi;

use crate::board::pins::SdCardPins;
use crate::error::AppResult;
use fatfs::MountedCard;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};

/// Расширения файлов, распознаваемые как файлы G-Code.
const GCODE_EXTENSIONS: [&str; 2] = ["gcode", "gco"];

/// Карта памяти, смонтированная и готовая к чтению G-Code файлов.
pub struct SdCard {
    card: MountedCard,
}

impl SdCard {
    /// Инициализирует шину SPI и монтирует карту памяти, используя
    /// распиновку станка ([`SdCardPins`]).
    pub fn mount(pins: &SdCardPins, max_open_files: usize, format_if_mount_failed: bool) -> AppResult<Self> {
        spi::init_bus(pins)?;
        let card = MountedCard::mount(pins.cs, "/sdcard", max_open_files, format_if_mount_failed)?;
        Ok(Self { card })
    }

    /// Перечисляет файлы G-Code в корне карты памяти, отсортированные по
    /// имени. Вложенные директории не сканируются рекурсивно на этом
    /// этапе — большинство слайсеров кладут файлы в корень или в одну
    /// выбранную пользователем папку.
    pub fn list_gcode_files(&self) -> AppResult<Vec<String>> {
        list_gcode_files_in(Path::new(self.card.mount_path()))
    }

    /// Открывает файл `name` (относительно корня карты) для построчного
    /// чтения G-Code.
    pub fn open_gcode_file(&self, name: &str) -> AppResult<GcodeFileReader> {
        let path = PathBuf::from(self.card.mount_path()).join(name);
        GcodeFileReader::open(&path)
    }
}

/// Перечисляет файлы с расширением G-Code в директории `dir` — вынесено из
/// [`SdCard::list_gcode_files`] в свободную функцию, чтобы логику фильтрации
/// можно было протестировать на обычной временной директории хоста, не
/// монтируя настоящую карту памяти.
fn list_gcode_files_in(dir: &Path) -> AppResult<Vec<String>> {
    let mut files = Vec::new();
    for entry in std::fs::read_dir(dir)? {
        let entry = entry?;
        if !entry.file_type()?.is_file() {
            continue;
        }
        let path = entry.path();
        let is_gcode = path
            .extension()
            .and_then(|ext| ext.to_str())
            .map(|ext| GCODE_EXTENSIONS.iter().any(|allowed| ext.eq_ignore_ascii_case(allowed)))
            .unwrap_or(false);
        if is_gcode {
            if let Some(name) = path.file_name().and_then(|n| n.to_str()) {
                files.push(name.to_string());
            }
        }
    }
    files.sort();
    Ok(files)
}

/// Построчное чтение G-Code файла с отслеживанием прогресса по байтам.
pub struct GcodeFileReader {
    reader: BufReader<File>,
    total_bytes: u64,
    bytes_read: u64,
}

impl GcodeFileReader {
    /// Открывает файл для чтения, запоминая его полный размер для расчёта
    /// прогресса печати.
    pub fn open(path: &Path) -> AppResult<Self> {
        let file = File::open(path)?;
        let total_bytes = file.metadata()?.len();
        Ok(Self {
            reader: BufReader::new(file),
            total_bytes,
            bytes_read: 0,
        })
    }

    /// Читает следующую строку файла (без завершающего перевода строки).
    /// Возвращает `Ok(None)` по достижении конца файла.
    pub fn next_line(&mut self) -> AppResult<Option<String>> {
        let mut line = String::new();
        let bytes = self.reader.read_line(&mut line)?;
        if bytes == 0 {
            return Ok(None);
        }
        self.bytes_read += bytes as u64;
        while line.ends_with('\n') || line.ends_with('\r') {
            line.pop();
        }
        Ok(Some(line))
    }

    /// Доля прочитанного файла, `0.0..=1.0` (`1.0`, если файл пуст).
    #[must_use]
    pub fn progress_fraction(&self) -> f32 {
        if self.total_bytes == 0 {
            1.0
        } else {
            (self.bytes_read as f64 / self.total_bytes as f64) as f32
        }
    }
}

/// Состояние задания печати с карты памяти.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PrintJobState {
    /// Печать не запущена.
    Idle,
    /// Идёт построчная подача команд исполнителю G-Code.
    Printing,
    /// Приостановлена пользователем (`M25`-подобное действие на уровне
    /// приложения — конкретная команда, вызывающая паузу, не входит в
    /// обязательный список G-Code этого ТЗ и подключается на уровне `App`).
    Paused,
    /// Печать завершена успешно (достигнут конец файла).
    Completed,
    /// Печать прервана пользователем или из-за ошибки.
    Aborted,
}

/// Задание печати с карты памяти: читает файл построчно и отдаёт по одной
/// команде за вызов [`PrintJob::next_command`] — фактическая передача
/// команды в [`crate::gcode::executor::GcodeExecutor`] выполняется
/// вызывающим кодом, чтобы `PrintJob` не зависел от конкретного типа
/// исполнителя (`PrinterContext`).
pub struct PrintJob {
    file_name: String,
    reader: GcodeFileReader,
    state: PrintJobState,
    lines_sent: u64,
}

impl PrintJob {
    /// Начинает новое задание печати из уже открытого файла.
    #[must_use]
    pub fn start(file_name: String, reader: GcodeFileReader) -> Self {
        Self {
            file_name,
            reader,
            state: PrintJobState::Printing,
            lines_sent: 0,
        }
    }

    /// Имя печатаемого файла.
    #[must_use]
    pub fn file_name(&self) -> &str {
        &self.file_name
    }

    /// Текущее состояние задания.
    #[must_use]
    pub fn state(&self) -> PrintJobState {
        self.state
    }

    /// Приостанавливает печать. Не читает новых строк, пока не будет
    /// вызван [`PrintJob::resume`].
    pub fn pause(&mut self) {
        if self.state == PrintJobState::Printing {
            self.state = PrintJobState::Paused;
        }
    }

    /// Возобновляет ранее приостановленную печать.
    pub fn resume(&mut self) {
        if self.state == PrintJobState::Paused {
            self.state = PrintJobState::Printing;
        }
    }

    /// Прерывает печать. Дальнейшие вызовы [`PrintJob::next_command`]
    /// всегда будут возвращать `Ok(None)`.
    pub fn abort(&mut self) {
        self.state = PrintJobState::Aborted;
    }

    /// Возвращает следующую непустую строку файла для исполнения, либо
    /// `Ok(None)`, если печать приостановлена/завершена/прервана, либо
    /// файл закончился (в последнем случае состояние переводится в
    /// [`PrintJobState::Completed`]).
    ///
    /// Пустые строки и строки-комментарии файла пропускаются на этом
    /// уровне не полностью — фильтрация пустого содержимого после
    /// удаления комментариев остаётся на совести
    /// [`crate::gcode::parser::parse_line`], который и так возвращает
    /// `Ok(None)` для таких строк.
    pub fn next_command(&mut self) -> AppResult<Option<String>> {
        if self.state != PrintJobState::Printing {
            return Ok(None);
        }

        match self.reader.next_line()? {
            Some(line) => {
                self.lines_sent += 1;
                Ok(Some(line))
            }
            None => {
                self.state = PrintJobState::Completed;
                Ok(None)
            }
        }
    }

    /// Доля выполненного задания, `0.0..=1.0`, по прочитанным байтам файла.
    #[must_use]
    pub fn progress_fraction(&self) -> f32 {
        self.reader.progress_fraction()
    }

    /// Количество строк, уже отданных на исполнение.
    #[must_use]
    pub fn lines_sent(&self) -> u64 {
        self.lines_sent
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    fn write_temp_file(dir: &Path, name: &str, contents: &str) -> PathBuf {
        let path = dir.join(name);
        let mut file = File::create(&path).unwrap();
        file.write_all(contents.as_bytes()).unwrap();
        path
    }

    fn temp_dir(suffix: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("osix-sdcard-test-{suffix}-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn lists_only_gcode_files_sorted_by_name() {
        let dir = temp_dir("list");
        write_temp_file(&dir, "b.gcode", "G28\n");
        write_temp_file(&dir, "a.gco", "G28\n");
        write_temp_file(&dir, "readme.txt", "not gcode");

        let files = list_gcode_files_in(&dir).unwrap();
        assert_eq!(files, vec!["a.gco".to_string(), "b.gcode".to_string()]);

        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn gcode_file_reader_strips_newlines_and_tracks_progress() {
        let dir = temp_dir("reader");
        let path = write_temp_file(&dir, "test.gcode", "G28\nG1 X10\n");

        let mut reader = GcodeFileReader::open(&path).unwrap();
        assert_eq!(reader.next_line().unwrap(), Some("G28".to_string()));
        assert!(reader.progress_fraction() > 0.0 && reader.progress_fraction() < 1.0);
        assert_eq!(reader.next_line().unwrap(), Some("G1 X10".to_string()));
        assert_eq!(reader.next_line().unwrap(), None);
        assert!((reader.progress_fraction() - 1.0).abs() < 1e-6);

        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn print_job_completes_after_last_line() {
        let dir = temp_dir("job-complete");
        let path = write_temp_file(&dir, "job.gcode", "G28\nG1 X1\n");
        let reader = GcodeFileReader::open(&path).unwrap();
        let mut job = PrintJob::start("job.gcode".to_string(), reader);

        assert_eq!(job.next_command().unwrap(), Some("G28".to_string()));
        assert_eq!(job.state(), PrintJobState::Printing);
        assert_eq!(job.next_command().unwrap(), Some("G1 X1".to_string()));
        assert_eq!(job.next_command().unwrap(), None);
        assert_eq!(job.state(), PrintJobState::Completed);
        assert_eq!(job.lines_sent(), 2);

        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn print_job_pause_blocks_next_command_until_resumed() {
        let dir = temp_dir("job-pause");
        let path = write_temp_file(&dir, "job.gcode", "G28\nG1 X1\n");
        let reader = GcodeFileReader::open(&path).unwrap();
        let mut job = PrintJob::start("job.gcode".to_string(), reader);

        job.pause();
        assert_eq!(job.next_command().unwrap(), None, "на паузе новые строки не выдаются");

        job.resume();
        assert_eq!(job.next_command().unwrap(), Some("G28".to_string()));

        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn print_job_abort_is_terminal() {
        let dir = temp_dir("job-abort");
        let path = write_temp_file(&dir, "job.gcode", "G28\n");
        let reader = GcodeFileReader::open(&path).unwrap();
        let mut job = PrintJob::start("job.gcode".to_string(), reader);

        job.abort();
        assert_eq!(job.next_command().unwrap(), None);
        job.resume(); // не должно "воскрешать" прерванное задание
        assert_eq!(job.state(), PrintJobState::Aborted);

        std::fs::remove_dir_all(&dir).ok();
    }
}
