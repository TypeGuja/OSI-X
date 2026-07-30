//! Обёртка над задачами FreeRTOS.
//!
//! ESP-IDF в связке со `std` отображает `std::thread` на задачи FreeRTOS
//! напрямую, поэтому отдельный примитив создания задач не нужен — но
//! приоритет и привязку к ядру для `std::thread` нужно задать заранее через
//! `esp_idf_hal::task::thread::ThreadSpawnConfiguration`, что и делает
//! [`spawn`].

use crate::error::{AppError, AppResult};
use esp_idf_hal::task::thread::ThreadSpawnConfiguration;
use std::thread::JoinHandle;

/// Приоритет задачи FreeRTOS. Значения выбраны так, чтобы генератор шагов
/// всегда вытеснял менее критичные по времени подсистемы (сеть, G-Code).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TaskPriority {
    /// Генератор шагов — самый высокий приоритет: пропуск тайминга шага
    /// напрямую портит геометрию печати.
    StepGenerator,
    /// Регулирование температуры (PID, thermal runaway) — должно вытеснять
    /// сеть и G-Code, но не мешать генератору шагов.
    Temperature,
    /// Обработчик G-Code, планировщик движения верхнего уровня.
    GCode,
    /// Сетевые сервисы (HTTP, WebSocket) — наименее критичны по времени.
    Network,
}

impl TaskPriority {
    /// Числовой приоритет FreeRTOS (`0` — самый низкий; ESP-IDF по
    /// умолчанию использует диапазон `0..=24`).
    const fn to_freertos_priority(self) -> u8 {
        match self {
            TaskPriority::StepGenerator => 20,
            TaskPriority::Temperature => 15,
            TaskPriority::GCode => 10,
            TaskPriority::Network => 5,
        }
    }
}

/// Дескриптор запущенной задачи.
pub struct Task {
    name: &'static str,
    join_handle: Option<JoinHandle<()>>,
}

impl Task {
    /// Запускает новую задачу FreeRTOS с именем `name`, приоритетом
    /// `priority` и размером стека `stack_size_bytes`, выполняющую `body`.
    ///
    /// `body` не должна завершаться в обычной эксплуатации (все задачи
    /// прошивки — бесконечные циклы); если она всё же вернётся, задача
    /// просто завершится штатным образом FreeRTOS.
    pub fn spawn(
        name: &'static str,
        priority: TaskPriority,
        stack_size_bytes: usize,
        body: impl FnOnce() + Send + 'static,
    ) -> AppResult<Self> {
        let config = ThreadSpawnConfiguration {
            name: Some(name.as_bytes()),
            stack_size: stack_size_bytes,
            priority: priority.to_freertos_priority(),
            ..Default::default()
        };
        config
            .set()
            .map_err(|e| AppError::board(format!("не удалось настроить параметры задачи '{name}': {e}")))?;

        let join_handle = std::thread::Builder::new()
            .name(name.to_string())
            .stack_size(stack_size_bytes)
            .spawn(body)
            .map_err(|e| AppError::board(format!("не удалось запустить задачу '{name}': {e}")))?;

        // Настройка `ThreadSpawnConfiguration` применяется только к
        // следующему создаваемому потоку — сбрасываем её сразу после
        // использования, чтобы не повлиять на последующие вызовы `spawn`
        // в других частях прошивки.
        ThreadSpawnConfiguration::default()
            .set()
            .map_err(|e| AppError::board(format!("не удалось сбросить конфигурацию задачи: {e}")))?;

        log::info!("задача '{name}' запущена (приоритет {:?}, стек {stack_size_bytes} байт)", priority);

        Ok(Self {
            name,
            join_handle: Some(join_handle),
        })
    }

    /// Имя задачи.
    #[must_use]
    pub fn name(&self) -> &'static str {
        self.name
    }

    /// Блокирующе дожидается завершения задачи (в штатной работе не
    /// вызывается — задачи прошивки рассчитаны на бесконечное выполнение).
    pub fn join(mut self) -> AppResult<()> {
        if let Some(handle) = self.join_handle.take() {
            handle
                .join()
                .map_err(|_| AppError::board(format!("задача '{}' завершилась с паникой", self.name)))?;
        }
        Ok(())
    }
}
