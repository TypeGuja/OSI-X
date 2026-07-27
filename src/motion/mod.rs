//! Подсистема движения станка: кинематика, планировщик с look-ahead,
//! трапецеидальные профили скорости и генератор шагов.
//!
//! Поток данных: `gcode::executor` вызывает
//! [`planner::MotionPlanner::plan_linear_move`] с целевой точкой в
//! координатах эффектора → планировщик переводит её через [`kinematics`] в
//! координаты логических осей, кладёт сегмент в [`queue::MotionQueue`] и
//! пересчитывает допустимые скорости стыков для всей очереди → отдельная
//! задача планировщика ([`crate::scheduler`]) извлекает готовые сегменты и
//! передаёт их в [`step_generator::StepGenerator`], который строит
//! [`trapezoid::TrapezoidProfile`] и синхронно шагает всеми осями.
//!
//! `dead_code` временно отключён: модуль полностью реализован и покрыт
//! тестами, но ещё не вызывается из `App` — это произойдёт на этапе
//! подключения `gcode::executor`, который создаёт `MotionPlanner` и
//! задачу генератора шагов.

pub mod acceleration;
pub mod kinematics;
pub mod planner;
pub mod queue;
pub mod step_generator;
pub mod trapezoid;

pub use kinematics::{AxisPosition, CartesianPosition, Kinematics};
pub use planner::MotionPlanner;
pub use queue::{MotionQueue, MotionSegment};
pub use step_generator::{EtsStepClock, StepClock, StepGenerator};
pub use trapezoid::TrapezoidProfile;
