//! Общие типы, используемые несколькими подсистемами прошивки.
//!
//! Здесь находятся только "сквозные" типы (идентификаторы осей, единицы
//! измерения, направление вращения), не привязанные к конкретному драйверу
//! или подсистеме. Специфичные типы (например, регистры TMC2209) живут
//! рядом со своими модулями.

use serde::{Deserialize, Serialize};
use std::fmt;

/// Идентификатор оси станка.
///
/// Список сознательно сделан расширяемым: на первом этапе используются
/// только `X`, `Y`, `Z`, но кинематика CoreXY/Delta и мульти-экструдеры
/// потребуют дополнительных осей (`E0`, `E1`, ...), которые будут добавлены
/// без изменения остального кода благодаря тому, что `AxisId` уже сейчас
/// используется как ключ, а не как позиционный индекс.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum AxisId {
    /// Ось X.
    X,
    /// Ось Y.
    Y,
    /// Ось Z.
    Z,
}

impl AxisId {
    /// Все оси, поддерживаемые на текущем этапе, в порядке инициализации.
    pub const ALL: [AxisId; 3] = [AxisId::X, AxisId::Y, AxisId::Z];
}

impl fmt::Display for AxisId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            AxisId::X => "X",
            AxisId::Y => "Y",
            AxisId::Z => "Z",
        };
        write!(f, "{s}")
    }
}

/// Направление вращения двигателя относительно "положительного" направления
/// оси, заданного в конфигурации.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorDirection {
    /// Движение в положительном направлении оси.
    Forward,
    /// Движение в отрицательном направлении оси.
    Backward,
}

impl MotorDirection {
    /// Возвращает направление, противоположное текущему.
    #[must_use]
    pub const fn reversed(self) -> Self {
        match self {
            MotorDirection::Forward => MotorDirection::Backward,
            MotorDirection::Backward => MotorDirection::Forward,
        }
    }
}

/// Расстояние в миллиметрах (используется в кинематике и планировщике).
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct Millimeters(pub f32);

/// Скорость в миллиметрах в секунду.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct MmPerSecond(pub f32);

/// Ускорение в миллиметрах в секунду в квадрате.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct MmPerSecondSquared(pub f32);

/// Целочисленное количество шагов двигателя (может быть отрицательным —
/// знак задаёт направление до применения [`MotorDirection`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Steps(pub i64);

/// Длительность в микросекундах, используемая генератором шагов и
/// планировщиком (более естественная единица для тайминга степов, чем
/// `std::time::Duration`, которая избыточна для горячего пути).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Microseconds(pub u64);

impl Microseconds {
    /// Создаёт значение из частоты в герцах (0 Гц трактуется как максимально
    /// возможный период, чтобы избежать деления на ноль в вызывающем коде).
    #[must_use]
    pub fn from_hz(hz: f32) -> Self {
        if hz <= 0.0 {
            Self(u64::MAX)
        } else {
            Self((1_000_000.0 / hz) as u64)
        }
    }
}

/// Электрический ток в миллиамперах (RMS), используется конфигурацией
/// драйверов моторов (например, `IHOLD`/`IRUN` TMC2209).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct Milliamps(pub u16);
