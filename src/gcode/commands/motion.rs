//! Обработчики команд движения: `G0`, `G1`, `G4`, `G28`, `G90`, `G91`,
//! `G92`, `M17`, `M18`.

use crate::error::AppResult;
use crate::gcode::commands::{AxisSelector, GcodeState, PositioningMode, PrinterContext};
use crate::gcode::parser::GcodeCommand;
use crate::motion::CartesianPosition;

/// Переводит скорость подачи из мм/мин (единица G-Code, параметр `F`) в
/// мм/с (единица, используемая планировщиком движения).
fn feed_rate_mm_per_min_to_mm_per_s(feed_rate_mm_per_min: f32) -> f32 {
    feed_rate_mm_per_min / 60.0
}

/// Вычисляет целевую координату одной оси с учётом режима позиционирования:
/// абсолютное значение параметра, смещение относительно текущей позиции,
/// либо (если параметр отсутствует) — без изменений.
fn resolve_axis_target(current: f32, param: Option<f32>, mode: PositioningMode) -> f32 {
    match (param, mode) {
        (Some(value), PositioningMode::Absolute) => value,
        (Some(delta), PositioningMode::Relative) => current + delta,
        (None, _) => current,
    }
}

/// Обрабатывает `G0`/`G1` — линейное перемещение (с точки зрения текущей
/// аппаратной конфигурации, без экструдера, `G0` и `G1` эквивалентны;
/// параметр `E`, если присутствует в сляйсированном G-Code, принимается и
/// намеренно игнорируется — см. примечание в [`crate::gcode::executor`]).
pub fn handle_linear_move<C: PrinterContext>(
    context: &mut C,
    state: &mut GcodeState,
    command: &GcodeCommand,
) -> AppResult<Option<String>> {
    if let Some(feed_rate) = command.get('F') {
        state.feed_rate_mm_s = feed_rate_mm_per_min_to_mm_per_s(feed_rate).max(0.0);
    }

    let current = context.current_position();
    let target = CartesianPosition {
        x: resolve_axis_target(current.x, command.get('X'), state.positioning_mode),
        y: resolve_axis_target(current.y, command.get('Y'), state.positioning_mode),
        z: resolve_axis_target(current.z, command.get('Z'), state.positioning_mode),
    };

    context.plan_linear_move(target, state.feed_rate_mm_s)?;
    Ok(None)
}

/// Обрабатывает `G4` — пауза на заданное время. Параметр `P` — миллисекунды,
/// `S` — секунды (если присутствуют оба, `P` имеет приоритет).
pub fn handle_dwell<C: PrinterContext>(context: &mut C, command: &GcodeCommand) -> AppResult<Option<String>> {
    let milliseconds = if let Some(p) = command.get('P') {
        p.max(0.0) as u32
    } else if let Some(s) = command.get('S') {
        (s.max(0.0) * 1000.0) as u32
    } else {
        0
    };

    context.delay_ms(milliseconds);
    Ok(None)
}

/// Разбирает набор осей, указанных буквами параметров команды (`X`, `Y`,
/// `Z`) — используется `G28`, `M17`, `M18`. Если ни одна ось не указана,
/// возвращает выбор всех осей (стандартное поведение RepRap: `G28` без
/// параметров означает "хоуминг всех осей").
fn axis_selector_from_command(command: &GcodeCommand) -> AxisSelector {
    let selector = AxisSelector {
        x: command.has('X'),
        y: command.has('Y'),
        z: command.has('Z'),
    };
    if selector.any_selected() {
        selector
    } else {
        AxisSelector::all()
    }
}

/// Обрабатывает `G28` — хоуминг выбранных осей (или всех, если ни одна не
/// указана явно).
pub fn handle_home<C: PrinterContext>(context: &mut C, command: &GcodeCommand) -> AppResult<Option<String>> {
    let axes = axis_selector_from_command(command);
    context.home_axes(axes)?;
    Ok(None)
}

/// Обрабатывает `G90` — переключение в абсолютный режим позиционирования.
pub fn handle_absolute_positioning(state: &mut GcodeState) -> AppResult<Option<String>> {
    state.positioning_mode = PositioningMode::Absolute;
    Ok(None)
}

/// Обрабатывает `G91` — переключение в относительный режим позиционирования.
pub fn handle_relative_positioning(state: &mut GcodeState) -> AppResult<Option<String>> {
    state.positioning_mode = PositioningMode::Relative;
    Ok(None)
}

/// Обрабатывает `G92` — устанавливает текущую позицию без физического
/// перемещения (например, `G92 X0 Y0` обнуляет систему координат).
pub fn handle_set_position<C: PrinterContext>(context: &mut C, command: &GcodeCommand) -> AppResult<Option<String>> {
    let current = context.current_position();
    let new_position = CartesianPosition {
        x: command.get('X').unwrap_or(current.x),
        y: command.get('Y').unwrap_or(current.y),
        z: command.get('Z').unwrap_or(current.z),
    };
    context.set_current_position(new_position);
    Ok(None)
}

/// Обрабатывает `M17` — включение моторов выбранных (или всех) осей.
pub fn handle_enable_motors<C: PrinterContext>(context: &mut C, command: &GcodeCommand) -> AppResult<Option<String>> {
    context.enable_motors(axis_selector_from_command(command))?;
    Ok(None)
}

/// Обрабатывает `M18` — выключение моторов выбранных (или всех) осей.
pub fn handle_disable_motors<C: PrinterContext>(context: &mut C, command: &GcodeCommand) -> AppResult<Option<String>> {
    context.disable_motors(axis_selector_from_command(command))?;
    Ok(None)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn feed_rate_conversion_matches_known_value() {
        // 6000 мм/мин = 100 мм/с.
        assert!((feed_rate_mm_per_min_to_mm_per_s(6000.0) - 100.0).abs() < 1e-6);
    }

    #[test]
    fn resolve_axis_target_absolute_uses_param_value() {
        assert_eq!(resolve_axis_target(10.0, Some(25.0), PositioningMode::Absolute), 25.0);
    }

    #[test]
    fn resolve_axis_target_relative_adds_to_current() {
        assert_eq!(resolve_axis_target(10.0, Some(5.0), PositioningMode::Relative), 15.0);
    }

    #[test]
    fn resolve_axis_target_missing_param_keeps_current() {
        assert_eq!(resolve_axis_target(10.0, None, PositioningMode::Absolute), 10.0);
        assert_eq!(resolve_axis_target(10.0, None, PositioningMode::Relative), 10.0);
    }

    #[test]
    fn axis_selector_defaults_to_all_when_none_specified() {
        let command = crate::gcode::parser::parse_line("G28").unwrap().unwrap();
        let selector = axis_selector_from_command(&command);
        assert_eq!(selector, AxisSelector::all());
    }

    #[test]
    fn axis_selector_respects_explicit_axes() {
        let command = crate::gcode::parser::parse_line("G28 X Z").unwrap().unwrap();
        let selector = axis_selector_from_command(&command);
        assert!(selector.x);
        assert!(!selector.y);
        assert!(selector.z);
    }
}
