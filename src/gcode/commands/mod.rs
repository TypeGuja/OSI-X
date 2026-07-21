//! Общие типы и трейт [`PrinterContext`] — граница абстракции между
//! исполнителем G-Code ([`crate::gcode::executor`]) и остальными
//! подсистемами станка (движение, температура, система).
//!
//! Как [`crate::drivers::motor::driver::MotorDriver`] абстрагирует
//! исполнителя движения от конкретного драйвера двигателя,
//! [`PrinterContext`] абстрагирует исполнитель G-Code от конкретной
//! реализации `App` — это позволяет реализовать и протестировать парсер и
//! диспетчеризацию команд ещё до того, как модули `temperature` и
//! `storage` будут подключены к `App` на последующих этапах.

pub mod motion;
pub mod system;
pub mod temperature;

use crate::error::AppResult;
use crate::motion::CartesianPosition;

/// Выбор подмножества осей X/Y/Z — используется `G28` (хоуминг) и
/// `M17`/`M18` (включение/выключение моторов).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct AxisSelector {
    /// Выбрана ось X.
    pub x: bool,
    /// Выбрана ось Y.
    pub y: bool,
    /// Выбрана ось Z.
    pub z: bool,
}

impl AxisSelector {
    /// Выбор всех трёх осей.
    #[must_use]
    pub const fn all() -> Self {
        Self { x: true, y: true, z: true }
    }

    /// Возвращает `true`, если выбрана хотя бы одна ось.
    #[must_use]
    pub const fn any_selected(&self) -> bool {
        self.x || self.y || self.z
    }
}

/// Режим интерпретации координат в командах перемещения.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PositioningMode {
    /// `G90` — координаты абсолютные.
    Absolute,
    /// `G91` — координаты относительно текущей позиции.
    Relative,
}

/// Изменяемое состояние исполнителя G-Code между командами (не относится к
/// физическому состоянию станка — только к интерпретации последующих
/// команд: режим позиционирования, последняя скорость подачи).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GcodeState {
    /// Текущий режим позиционирования (`G90`/`G91`).
    pub positioning_mode: PositioningMode,
    /// Последняя установленная скорость подачи, мм/с (используется, если
    /// команда перемещения не содержит параметр `F`).
    pub feed_rate_mm_s: f32,
}

impl Default for GcodeState {
    fn default() -> Self {
        Self {
            positioning_mode: PositioningMode::Absolute,
            feed_rate_mm_s: 50.0,
        }
    }
}

/// Информация о прошивке, возвращаемая по `M115`.
#[derive(Debug, Clone, PartialEq)]
pub struct FirmwareInfo {
    /// Имя прошивки.
    pub firmware_name: &'static str,
    /// Версия прошивки.
    pub firmware_version: &'static str,
    /// Имя используемой кинематической схемы (см. [`crate::motion::Kinematics::name`]).
    pub kinematics_name: &'static str,
    /// Количество экструдеров.
    pub extruder_count: u8,
}

impl FirmwareInfo {
    /// Форматирует информацию в строку, совместимую по духу с ответом
    /// `M115` прошивок семейства Marlin/RepRap.
    #[must_use]
    pub fn to_report_string(&self) -> String {
        format!(
            "FIRMWARE_NAME:{} FIRMWARE_VERSION:{} KINEMATICS:{} EXTRUDER_COUNT:{}",
            self.firmware_name, self.firmware_version, self.kinematics_name, self.extruder_count
        )
    }
}

/// Состояние концевых выключателей, возвращаемое по `M119`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EndstopStates {
    /// Концевик оси X сработал.
    pub x_triggered: bool,
    /// Концевик оси Y сработал.
    pub y_triggered: bool,
    /// Концевик оси Z сработал.
    pub z_triggered: bool,
}

impl EndstopStates {
    /// Форматирует состояние в строку, совместимую по духу с ответом
    /// `M119` прошивок семейства Marlin.
    #[must_use]
    pub fn to_report_string(&self) -> String {
        format!(
            "x_min: {}\ny_min: {}\nz_min: {}",
            triggered_label(self.x_triggered),
            triggered_label(self.y_triggered),
            triggered_label(self.z_triggered),
        )
    }
}

fn triggered_label(triggered: bool) -> &'static str {
    if triggered {
        "TRIGGERED"
    } else {
        "open"
    }
}

/// Граница абстракции между исполнителем G-Code и станком.
///
/// Реализуется структурой верхнего уровня (`App` или посвящённый ей тип на
/// этапе финальной сборки прошивки), объединяющей [`crate::motion::MotionPlanner`],
/// подсистему температуры и хранилище настроек. Исполнитель G-Code видит
/// только этот трейт и не завязан на конкретный набор подсистем.
pub trait PrinterContext {
    // --- Движение ---------------------------------------------------

    /// Ставит в очередь линейное перемещение к абсолютной точке `target` с
    /// заданной скоростью подачи. Может не выполниться немедленно —
    /// реализация вправе поставить сегмент в очередь планировщика и
    /// вернуть управление, не дожидаясь физического завершения движения.
    fn plan_linear_move(&mut self, target: CartesianPosition, feed_rate_mm_s: f32) -> AppResult<()>;

    /// Текущее известное (запланированное) положение эффектора.
    fn current_position(&self) -> CartesianPosition;

    /// Принудительно устанавливает текущее положение без движения (`G92`).
    fn set_current_position(&mut self, position: CartesianPosition);

    /// Выполняет хоуминг выбранных осей.
    fn home_axes(&mut self, axes: AxisSelector) -> AppResult<()>;

    /// Включает моторы выбранных осей (`M17`).
    fn enable_motors(&mut self, axes: AxisSelector) -> AppResult<()>;

    /// Выключает моторы выбранных осей (`M18`).
    fn disable_motors(&mut self, axes: AxisSelector) -> AppResult<()>;

    /// Блокирующая пауза на заданное число миллисекунд (`G4`).
    fn delay_ms(&mut self, milliseconds: u32);

    // --- Температура -------------------------------------------------

    /// Устанавливает целевую температуру хотэнда, °C (`M104`/`M109`).
    fn set_hotend_target(&mut self, celsius: f32) -> AppResult<()>;

    /// Текущая измеренная температура хотэнда, °C.
    fn hotend_temperature(&self) -> f32;

    /// Текущая целевая температура хотэнда, °C.
    fn hotend_target(&self) -> f32;

    /// Блокирует вызывающий поток до достижения целевой температуры
    /// хотэнда в пределах допуска, настроенного подсистемой температуры.
    fn wait_for_hotend_target(&mut self) -> AppResult<()>;

    /// Устанавливает целевую температуру стола, °C (`M140`/`M190`).
    fn set_bed_target(&mut self, celsius: f32) -> AppResult<()>;

    /// Текущая измеренная температура стола, °C.
    fn bed_temperature(&self) -> f32;

    /// Текущая целевая температура стола, °C.
    fn bed_target(&self) -> f32;

    /// Блокирует вызывающий поток до достижения целевой температуры стола.
    fn wait_for_bed_target(&mut self) -> AppResult<()>;

    /// Устанавливает скорость вентилятора обдува детали (`M106`/`M107`).
    /// `speed_0_255 == 0` соответствует `M107` (полная остановка).
    fn set_part_fan_speed(&mut self, speed_0_255: u8) -> AppResult<()>;

    // --- Система -------------------------------------------------------

    /// Информация о прошивке (`M115`).
    fn firmware_info(&self) -> FirmwareInfo;

    /// Состояние концевых выключателей (`M119`).
    fn endstop_states(&self) -> AppResult<EndstopStates>;

    /// Сохраняет текущие настройки в энергонезависимую память (`M500`).
    fn save_settings(&mut self) -> AppResult<()>;

    /// Загружает настройки из энергонезависимой памяти (`M501`).
    fn load_settings(&mut self) -> AppResult<()>;
}
