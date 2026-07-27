//! Подсистема G-Code: парсер, типы команд/состояния и исполнитель.
//!
//! Поток данных: сетевой/USB/SD-источник передаёт текстовые строки в
//! [`executor::GcodeExecutor::execute_line`], который парсит их через
//! [`parser::parse_line`] и диспетчеризует в обработчики из [`commands`].
//! Обработчики видят станок только через трейт [`commands::PrinterContext`]
//! — конкретная реализация (объединяющая `motion::MotionPlanner`,
//! подсистему температуры и хранилище настроек) появится при финальной
//! сборке `App` на одном из следующих этапов.
//!
//! `dead_code` временно отключён: модуль полностью реализован и покрыт
//! тестами (включая сквозные тесты `executor` с фиктивным
//! `PrinterContext`), но `App` пока не создаёт `GcodeExecutor` — это
//! произойдёт, когда появится реальная реализация `PrinterContext`,
//! объединяющая уже готовые `motion` и будущие `temperature`/`storage`.

pub mod commands;
pub mod executor;
pub mod parser;

pub use executor::GcodeExecutor;
pub use parser::{parse_line, GcodeCommand};
