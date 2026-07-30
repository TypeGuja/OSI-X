//! Диагностика памяти: свободная куча SRAM/PSRAM, исторический минимум
//! свободной памяти (индикатор пиковой нагрузки, полезен для выявления
//! утечек и подбора размеров стеков задач).

/// Снимок состояния кучи станка.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MemoryStats {
    /// Свободная память во всех доступных областях кучи прямо сейчас, байт.
    pub free_heap_bytes: u32,
    /// Наименьшее значение свободной памяти за всё время работы, байт —
    /// падение этого значения со временем при неизменной нагрузке
    /// указывает на утечку памяти.
    pub minimum_free_heap_bytes: u32,
    /// Свободная внутренняя SRAM (быстрая память, используется стеками
    /// задач и DMA-буферами), байт.
    pub free_internal_bytes: u32,
    /// Свободная внешняя PSRAM (8 МБ на ESP32-S3 N16R8, используется для
    /// больших буферов — например, очереди планировщика движения и кешей
    /// файлов SD-карты), байт.
    pub free_psram_bytes: u32,
}

/// Считывает текущее состояние кучи.
///
/// Все используемые функции (`esp_get_free_heap_size`,
/// `esp_get_minimum_free_heap_size`, `heap_caps_get_free_size`) — часть
/// стабильного публичного API ESP-IDF (`esp_heap_caps.h`), не подверженного
/// той версионной нестабильности, что затронула низкоуровневые структуры в
/// `sdcard`/`usb`.
#[must_use]
pub fn memory_stats() -> MemoryStats {
    // SAFETY: ни одна из вызываемых функций не принимает указателей и не
    // имеет предусловий — все они просто читают текущее состояние
    // аллокатора кучи ESP-IDF.
    unsafe {
        MemoryStats {
            free_heap_bytes: esp_idf_sys::esp_get_free_heap_size(),
            minimum_free_heap_bytes: esp_idf_sys::esp_get_minimum_free_heap_size(),
            free_internal_bytes: esp_idf_sys::heap_caps_get_free_size(esp_idf_sys::MALLOC_CAP_INTERNAL) as u32,
            free_psram_bytes: esp_idf_sys::heap_caps_get_free_size(esp_idf_sys::MALLOC_CAP_SPIRAM) as u32,
        }
    }
}

impl MemoryStats {
    /// Возвращает `true`, если свободной внутренней памяти осталось
    /// меньше `threshold_bytes` — сигнал для логирования предупреждения
    /// до того, как аллокация начнёт завершаться ошибкой в
    /// непредсказуемый момент.
    #[must_use]
    pub fn is_internal_memory_low(&self, threshold_bytes: u32) -> bool {
        self.free_internal_bytes < threshold_bytes
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn is_internal_memory_low_compares_against_threshold() {
        let stats = MemoryStats {
            free_heap_bytes: 100_000,
            minimum_free_heap_bytes: 50_000,
            free_internal_bytes: 8_000,
            free_psram_bytes: 4_000_000,
        };
        assert!(stats.is_internal_memory_low(10_000));
        assert!(!stats.is_internal_memory_low(4_000));
    }
}
