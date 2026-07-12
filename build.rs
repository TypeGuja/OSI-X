//! Сборочный скрипт проекта OSIX Firmware.
//!
//! Делегирует всю работу по интеграции с ESP-IDF crate'у `embuild`:
//! он генерирует биндинги, настраивает линковку и пробрасывает
//! переменные окружения ESP-IDF (`sdkconfig`, компоненты и т.д.)
//! в `esp-idf-sys`.

fn main() -> anyhow::Result<()> {
    embuild::espidf::sysenv::output();
    Ok(())
}
