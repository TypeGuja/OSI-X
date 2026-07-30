//! Планирование задач FreeRTOS: приоритеты, запуск задач ([`task`]) и
//! периодические программные таймеры ([`timer`]).
//!
//! `dead_code` временно отключён: `Task`/`PeriodicTimer` уже полностью
//! реализованы, но будут востребованы начиная с этапа подключения
//! `motion`/`gcode`/`temperature` к `App` (создание выделенных задач для
//! генератора шагов, исполнителя G-Code и опроса температуры).
#![allow(dead_code)]

pub mod task;
pub mod timer;

pub use task::{Task, TaskPriority};
pub use timer::PeriodicTimer;
