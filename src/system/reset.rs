//! Причина последнего сброса контроллера и программная перезагрузка.

use std::fmt;

/// Причина последнего сброса/включения контроллера.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResetReason {
    /// Штатное включение питания.
    PowerOn,
    /// Внешний сигнал сброса (кнопка RESET, вывод EN).
    ExternalPin,
    /// Программный перезапуск (`esp_restart`, в т.ч. после применения
    /// нового образа OTA).
    Software,
    /// Сброс из-за необработанной паники прошивки.
    Panic,
    /// Сработал сторожевой таймер прерываний (Interrupt Watchdog).
    InterruptWatchdog,
    /// Сработал Task Watchdog Timer (см. `board::watchdog`) — задача не
    /// сбросила счётчик вовремя.
    TaskWatchdog,
    /// Сработал иной сторожевой таймер (RTC WDT и т.п.).
    OtherWatchdog,
    /// Пробуждение из глубокого сна.
    DeepSleep,
    /// Сброс по снижению напряжения питания (brown-out) — частая причина
    /// на слаботочных источниках питания при одновременном старте моторов.
    BrownOut,
    /// Сброс, связанный с SDIO.
    Sdio,
    /// Сброс, инициированный через USB.
    Usb,
    /// Причина не распознана текущей версией ESP-IDF (запас на будущее).
    Unknown,
}

impl fmt::Display for ResetReason {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let text = match self {
            Self::PowerOn => "включение питания",
            Self::ExternalPin => "внешний сигнал сброса",
            Self::Software => "программный перезапуск",
            Self::Panic => "паника прошивки",
            Self::InterruptWatchdog => "сторожевой таймер прерываний",
            Self::TaskWatchdog => "Task Watchdog Timer",
            Self::OtherWatchdog => "иной сторожевой таймер",
            Self::DeepSleep => "пробуждение из глубокого сна",
            Self::BrownOut => "просадка напряжения питания (brown-out)",
            Self::Sdio => "сброс SDIO",
            Self::Usb => "сброс через USB",
            Self::Unknown => "неизвестная причина",
        };
        write!(f, "{text}")
    }
}

impl ResetReason {
    /// Возвращает `true` для причин, указывающих на аварийную ситуацию, а
    /// не на штатное включение/перезапуск — используется для решения,
    /// стоит ли перейти в безопасный режим при старте (например, не
    /// начинать хоуминг автоматически, если станок только что
    /// перезагрузился из-за паники или просадки питания).
    #[must_use]
    pub fn is_abnormal(self) -> bool {
        matches!(
            self,
            Self::Panic | Self::InterruptWatchdog | Self::TaskWatchdog | Self::OtherWatchdog | Self::BrownOut
        )
    }

    /// Разбирает значение `esp_reset_reason_t`.
    fn from_raw(raw: esp_idf_sys::esp_reset_reason_t) -> Self {
        match raw {
            esp_idf_sys::esp_reset_reason_t_ESP_RST_POWERON => Self::PowerOn,
            esp_idf_sys::esp_reset_reason_t_ESP_RST_EXT => Self::ExternalPin,
            esp_idf_sys::esp_reset_reason_t_ESP_RST_SW => Self::Software,
            esp_idf_sys::esp_reset_reason_t_ESP_RST_PANIC => Self::Panic,
            esp_idf_sys::esp_reset_reason_t_ESP_RST_INT_WDT => Self::InterruptWatchdog,
            esp_idf_sys::esp_reset_reason_t_ESP_RST_TASK_WDT => Self::TaskWatchdog,
            esp_idf_sys::esp_reset_reason_t_ESP_RST_WDT => Self::OtherWatchdog,
            esp_idf_sys::esp_reset_reason_t_ESP_RST_DEEPSLEEP => Self::DeepSleep,
            esp_idf_sys::esp_reset_reason_t_ESP_RST_BROWNOUT => Self::BrownOut,
            esp_idf_sys::esp_reset_reason_t_ESP_RST_SDIO => Self::Sdio,
            esp_idf_sys::esp_reset_reason_t_ESP_RST_USB => Self::Usb,
            _ => Self::Unknown,
        }
    }
}

/// Возвращает причину последнего сброса контроллера.
#[must_use]
pub fn reset_reason() -> ResetReason {
    // SAFETY: `esp_reset_reason` не принимает аргументов и не имеет
    // предусловий — читает значение, сохранённое загрузчиком ROM до
    // передачи управления прошивке.
    let raw = unsafe { esp_idf_sys::esp_reset_reason() };
    ResetReason::from_raw(raw)
}

/// Инициирует программную перезагрузку станка. Не возвращает управление.
///
/// Вызывающий код отвечает за корректное завершение критичных операций
/// перед вызовом (например, `Heater::disable`/`Power::disable`, чтобы не
/// оставить нагреватель или моторы включёнными на время перезагрузки) —
/// эта функция не выполняет никакой очистки сама.
pub fn restart() -> ! {
    log::warn!("инициирована программная перезагрузка станка (причина: запрос прошивки)");
    // SAFETY: не принимает аргументов; документированно никогда не
    // возвращает управление вызывающей стороне.
    unsafe {
        esp_idf_sys::esp_restart();
    }
    unreachable!("esp_restart() не должна возвращать управление")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn abnormal_reasons_are_flagged_correctly() {
        assert!(ResetReason::Panic.is_abnormal());
        assert!(ResetReason::TaskWatchdog.is_abnormal());
        assert!(ResetReason::BrownOut.is_abnormal());
        assert!(!ResetReason::PowerOn.is_abnormal());
        assert!(!ResetReason::Software.is_abnormal());
    }

    #[test]
    fn display_produces_non_empty_human_readable_text() {
        for reason in [
            ResetReason::PowerOn,
            ResetReason::ExternalPin,
            ResetReason::Software,
            ResetReason::Panic,
            ResetReason::InterruptWatchdog,
            ResetReason::TaskWatchdog,
            ResetReason::OtherWatchdog,
            ResetReason::DeepSleep,
            ResetReason::BrownOut,
            ResetReason::Sdio,
            ResetReason::Usb,
            ResetReason::Unknown,
        ] {
            assert!(!reason.to_string().is_empty());
        }
    }
}
