//! Системная диагностика: CPU ([`cpu`]), память ([`memory`]), паника
//! ([`panic`]) и причина/инициирование сброса ([`reset`]).
//!
//! `dead_code` временно отключён: модуль полностью реализован и покрыт
//! тестами там, где это возможно без реального железа (`memory`, `reset`),
//! но ещё не вызывается из `App` — `panic::install_panic_hook()` и сбор
//! `SystemStatus` для отчётности подключатся при финальной сборке
//! прошивки.
#![allow(dead_code)]

pub mod cpu;
pub mod memory;
pub mod panic;
pub mod reset;

use cpu::TaskInfo;
use memory::MemoryStats;
use reset::ResetReason;
use std::time::Duration;

/// Сводный снимок состояния станка для диагностики (логи при старте,
/// будущий `/api/status`, вывод в консоль по внутренней команде).
#[derive(Debug, Clone, PartialEq)]
pub struct SystemStatus {
    /// Время работы с момента включения.
    pub uptime: Duration,
    /// Тактовая частота CPU, МГц.
    pub cpu_frequency_mhz: u32,
    /// Число ядер CPU.
    pub core_count: u32,
    /// Снимок состояния памяти.
    pub memory: MemoryStats,
    /// Причина последнего сброса.
    pub reset_reason: ResetReason,
}

impl SystemStatus {
    /// Собирает снимок текущего состояния станка.
    #[must_use]
    pub fn collect() -> Self {
        Self {
            uptime: cpu::uptime(),
            cpu_frequency_mhz: cpu::cpu_frequency_mhz(),
            core_count: cpu::core_count(),
            memory: memory::memory_stats(),
            reset_reason: reset::reset_reason(),
        }
    }
}

/// Выполняет полную последовательность диагностической инициализации:
/// устанавливает обработчик паники и логирует причину последнего сброса.
///
/// Должна вызываться один раз, как можно раньше в `main`, сразу после
/// инициализации логирования — до неё сообщения о панике/сбросе некуда
/// записывать.
pub fn init() {
    panic::install_panic_hook();

    let reason = reset::reset_reason();
    if reason.is_abnormal() {
        log::warn!("предыдущий сброс станка был аварийным: {reason}");
    } else {
        log::info!("причина последнего сброса: {reason}");
    }
}

/// Возвращает список задач FreeRTOS вместе с общим статусом — удобная
/// точка входа для полной диагностики, объединяющая [`SystemStatus::collect`]
/// и [`cpu::list_tasks`], которые по отдельности могут завершаться с
/// разным успехом (список задач требует временного буфера и теоретически
/// может не поместиться при экстремальном количестве задач).
pub fn full_diagnostics() -> (SystemStatus, crate::error::AppResult<Vec<TaskInfo>>) {
    (SystemStatus::collect(), cpu::list_tasks())
}
