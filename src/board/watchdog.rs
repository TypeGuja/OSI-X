//! Обёртка над Task Watchdog Timer (TWDT) ESP-IDF.
//!
//! Прямых биндингов `esp_task_wdt_*` в `esp-idf-hal`/`esp-idf-svc` на момент
//! написания нет, поэтому модуль обращается к `esp-idf-sys` напрямую.
//! Это единственное место во всей прошивке, где это делается — весь
//! `unsafe`, необходимый для работы с watchdog, изолирован в данном файле.

use crate::error::{AppError, AppResult};
use esp_idf_sys::EspError;

/// Таймер аппаратного watchdog задач FreeRTOS.
///
/// При создании инициализирует TWDT с заданным таймаутом и подписывает
/// на него текущую задачу. Любая задача, добавленная через [`Watchdog::add_current_task`],
/// обязана периодически вызывать [`Watchdog::reset`] — иначе контроллер
/// будет перезагружен, что является намеренной защитой от зависаний
/// (например, в планировщике движения или обработчике G-Code).
pub struct Watchdog {
    timeout_s: u32,
}

impl Watchdog {
    /// Инициализирует TWDT с таймаутом `timeout_s` секунд и панику при
    /// срабатывании (`panic = true` приводит к немедленной перезагрузке,
    /// что предпочтительнее зависшего состояния станка с горячим соплом).
    pub fn init(timeout_s: u32) -> AppResult<Self> {
        let config = esp_idf_sys::esp_task_wdt_config_t {
            timeout_ms: timeout_s.saturating_mul(1000),
            idle_core_mask: (1 << esp_idf_sys::CONFIG_FREERTOS_NUMBER_OF_CORES) - 1,
            trigger_panic: true,
        };

        // SAFETY: `config` создан здесь и живёт на стеке до завершения вызова;
        // `esp_task_wdt_init` не сохраняет указатель после возврата (согласно
        // документации ESP-IDF), поэтому передача `&config` безопасна.
        let ret = unsafe { esp_idf_sys::esp_task_wdt_init(&config) };
        EspError::convert(ret)
            .map_err(|e| AppError::board(format!("не удалось инициализировать TWDT: {e}")))?;

        log::info!("Task Watchdog Timer инициализирован (таймаут {timeout_s} с)");
        Ok(Self { timeout_s })
    }

    /// Подписывает текущую задачу FreeRTOS на watchdog.
    ///
    /// После вызова этой функции задача обязана периодически вызывать
    /// [`Watchdog::reset`], иначе произойдёт перезагрузка контроллера.
    pub fn add_current_task(&self) -> AppResult<()> {
        // SAFETY: передаём `null`, что согласно документации ESP-IDF
        // означает "текущая задача"; функция не сохраняет указателей.
        let ret = unsafe { esp_idf_sys::esp_task_wdt_add(std::ptr::null_mut()) };
        EspError::convert(ret)
            .map_err(|e| AppError::board(format!("не удалось подписать задачу на TWDT: {e}")))?;
        Ok(())
    }

    /// Отписывает текущую задачу от watchdog (например, перед контролируемым
    /// удалением задачи).
    pub fn remove_current_task(&self) -> AppResult<()> {
        // SAFETY: аналогично `add_current_task` — `null` означает
        // "текущая задача", указатель не сохраняется вызываемой функцией.
        let ret = unsafe { esp_idf_sys::esp_task_wdt_delete(std::ptr::null_mut()) };
        EspError::convert(ret)
            .map_err(|e| AppError::board(format!("не удалось отписать задачу от TWDT: {e}")))?;
        Ok(())
    }

    /// Сбрасывает счётчик watchdog для текущей задачи ("кормит" собаку).
    ///
    /// Должна вызываться из каждой итерации главного цикла подписанной
    /// задачи (например, `scheduler`, обработчик G-Code).
    pub fn reset(&self) -> AppResult<()> {
        // SAFETY: функция не принимает указателей и не имеет побочных
        // эффектов, требующих дополнительных инвариантов с нашей стороны.
        let ret = unsafe { esp_idf_sys::esp_task_wdt_reset() };
        EspError::convert(ret)
            .map_err(|e| AppError::board(format!("не удалось сбросить TWDT: {e}")))?;
        Ok(())
    }

    /// Возвращает настроенный таймаут watchdog в секундах.
    #[must_use]
    pub fn timeout_seconds(&self) -> u32 {
        self.timeout_s
    }
}
