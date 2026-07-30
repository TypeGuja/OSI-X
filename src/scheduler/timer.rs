//! Периодический программный таймер поверх `esp_timer` (высокоразрешающий
//! системный таймер ESP-IDF), используемый для задач с фиксированным
//! периодом, не требующих микросекундной точности генератора шагов
//! (опрос термисторов, телеметрия WebSocket, обслуживание watchdog).
//!
//! Тайминг шаговых импульсов **не** использует этот таймер — для него
//! применяется блокирующий busy-wait [`crate::motion::step_generator::EtsStepClock`]
//! внутри выделенной задачи с максимальным приоритетом, поскольку
//! коллбэки `esp_timer` выполняются в общем системном таймерном контексте
//! и не гарантируют микросекундную точность при высокой нагрузке.

use crate::error::{AppError, AppResult};
use esp_idf_svc::timer::{EspTimer, EspTimerService, Task};
use std::time::Duration;

/// Периодический таймер, вызывающий заданный коллбэк с фиксированным
/// интервалом до тех пор, пока экземпляр [`PeriodicTimer`] не будет удалён.
pub struct PeriodicTimer {
    // Поле должно оставаться живым на всё время работы таймера — при
    // удалении `EspTimer` таймер ESP-IDF автоматически останавливается и
    // освобождается.
    _timer: EspTimer<'static>,
}

impl PeriodicTimer {
    /// Создаёт и немедленно запускает периодический таймер с периодом
    /// `period`, вызывающий `callback` в контексте системного таймера
    /// ESP-IDF (не в задаче — коллбэк должен быть быстрым и не блокирующим).
    pub fn start(period: Duration, callback: impl FnMut() + Send + 'static) -> AppResult<Self> {
        let service: EspTimerService<Task> = EspTimerService::new()
            .map_err(|e| AppError::board(format!("не удалось создать службу таймеров ESP-IDF: {e}")))?;

        let timer = service
            .timer(callback)
            .map_err(|e| AppError::board(format!("не удалось создать периодический таймер: {e}")))?;

        timer
            .every(period)
            .map_err(|e| AppError::board(format!("не удалось запустить периодический таймер: {e}")))?;

        Ok(Self { _timer: timer })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn duration_conversion_is_exact_for_whole_milliseconds() {
        // Регрессионный тест-заглушка: проверяет, что используемый нами
        // тип `Duration` не теряет точность на характерных периодах
        // (250 мс телеметрии, 20 с наблюдения thermal runaway).
        assert_eq!(Duration::from_millis(250).as_millis(), 250);
        assert_eq!(Duration::from_secs(20).as_millis(), 20_000);
    }
}
