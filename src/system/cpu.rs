//! Диагностика CPU: частота, число ядер, время работы, список задач FreeRTOS.

use crate::error::AppResult;
use std::ffi::CStr;
use std::time::Duration;

/// Текущая тактовая частота CPU, МГц.
#[must_use]
pub fn cpu_frequency_mhz() -> u32 {
    // SAFETY: `esp_clk_cpu_freq` не принимает аргументов и не имеет
    // предусловий — просто читает уже проинициализированное значение
    // тактового генератора.
    let hz = unsafe { esp_idf_sys::esp_clk_cpu_freq() };
    (hz / 1_000_000) as u32
}

/// Число ядер CPU, доступных FreeRTOS (`2` для ESP32-S3).
#[must_use]
pub const fn core_count() -> u32 {
    esp_idf_sys::CONFIG_FREERTOS_NUMBER_OF_CORES
}

/// Время работы станка с момента включения.
#[must_use]
pub fn uptime() -> Duration {
    // SAFETY: не принимает аргументов; возвращает микросекунды с момента
    // запуска высокоразрешающего таймера ESP-IDF (`esp_timer`), который
    // инициализируется до вызова `main`.
    let micros = unsafe { esp_idf_sys::esp_timer_get_time() };
    Duration::from_micros(micros.max(0) as u64)
}

/// Состояние задачи FreeRTOS.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TaskState {
    /// Выполняется прямо сейчас.
    Running,
    /// Готова к выполнению, ожидает выделения CPU.
    Ready,
    /// Заблокирована (ожидает событие, таймер, семафор и т.п.).
    Blocked,
    /// Приостановлена.
    Suspended,
    /// Удалена, но память ещё не освобождена.
    Deleted,
    /// Неизвестное/недокументированное состояние (запас на будущие версии
    /// FreeRTOS, не считается ошибкой).
    Unknown,
}

impl TaskState {
    fn from_raw(state: esp_idf_sys::eTaskState) -> Self {
        match state {
            esp_idf_sys::eTaskState_eRunning => Self::Running,
            esp_idf_sys::eTaskState_eReady => Self::Ready,
            esp_idf_sys::eTaskState_eBlocked => Self::Blocked,
            esp_idf_sys::eTaskState_eSuspended => Self::Suspended,
            esp_idf_sys::eTaskState_eDeleted => Self::Deleted,
            _ => Self::Unknown,
        }
    }
}

/// Диагностическая информация об одной задаче FreeRTOS.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TaskInfo {
    /// Имя задачи (как передано в `scheduler::Task::spawn`).
    pub name: String,
    /// Текущий приоритет.
    pub priority: u32,
    /// Состояние задачи.
    pub state: TaskState,
    /// Минимальный запас стека за всё время жизни задачи, слов (для
    /// ESP32-S3 — 4-байтных); малое значение сигнализирует о риске
    /// переполнения стека и требует увеличения `stack_size_bytes` при
    /// следующем вызове `scheduler::Task::spawn`.
    pub stack_high_water_mark_words: u32,
}

/// Возвращает диагностическую информацию обо всех задачах FreeRTOS,
/// запущенных на данный момент.
///
/// Примечание для проверки при первой сборке: `uxTaskGetSystemState` и тип
/// `TaskStatus_t` — стабильная часть публичного API FreeRTOS, но точные
/// имена полей в `esp-idf-sys` (`pcTaskName`, `eCurrentState`,
/// `uxCurrentPriority`, `usStackHighWaterMark`) стоит свериться при первой
/// сборке под конкретную версию.
pub fn list_tasks() -> AppResult<Vec<TaskInfo>> {
    // SAFETY: `uxTaskGetNumberOfTasks` не принимает аргументов и не имеет
    // предусловий.
    let task_count = unsafe { esp_idf_sys::uxTaskGetNumberOfTasks() } as usize;

    // Небольшой запас на случай, если между вызовами `uxTaskGetNumberOfTasks`
    // и `uxTaskGetSystemState` появится ещё одна задача.
    let capacity = task_count + 4;
    let mut statuses: Vec<esp_idf_sys::TaskStatus_t> =
        vec![unsafe { std::mem::zeroed() }; capacity];
    let mut total_run_time: u32 = 0;

    // SAFETY: `statuses` — валидный буфер длиной `capacity`, размер
    // передан корректно; `total_run_time` указывает на локальную
    // переменную для необязательного выходного значения.
    let filled = unsafe {
        esp_idf_sys::uxTaskGetSystemState(statuses.as_mut_ptr(), capacity as u32, &mut total_run_time)
    } as usize;

    statuses.truncate(filled);

    let tasks = statuses
        .into_iter()
        .map(|status| {
            // SAFETY: `pcTaskName` — указатель на статическую
            // NUL-терминированную строку, которую FreeRTOS хранит внутри
            // структуры управления задачей на всё время её жизни; на
            // момент чтения (сразу после `uxTaskGetSystemState`) задача
            // ещё существует.
            let name = unsafe { CStr::from_ptr(status.pcTaskName) }
                .to_string_lossy()
                .into_owned();

            TaskInfo {
                name,
                priority: status.uxCurrentPriority,
                state: TaskState::from_raw(status.eCurrentState),
                stack_high_water_mark_words: u32::from(status.usStackHighWaterMark),
            }
        })
        .collect();

    Ok(tasks)
}
