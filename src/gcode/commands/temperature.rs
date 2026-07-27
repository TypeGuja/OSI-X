//! Обработчики команд температуры и вентилятора: `M104`, `M105`, `M106`,
//! `M107`, `M109`, `M140`, `M190`.

use crate::error::AppResult;
use crate::gcode::commands::PrinterContext;
use crate::gcode::parser::GcodeCommand;

/// Скорость вентилятора по умолчанию для `M106` без параметра `S`
/// (соответствует полной скорости — стандартное поведение Marlin/RepRap).
const DEFAULT_FAN_SPEED: u8 = 255;

/// Обрабатывает `M104` — задаёт целевую температуру хотэнда без ожидания.
pub fn handle_set_hotend_temperature<C: PrinterContext>(
    context: &mut C,
    command: &GcodeCommand,
) -> AppResult<Option<String>> {
    if let Some(celsius) = command.get('S') {
        context.set_hotend_target(celsius)?;
    }
    Ok(None)
}

/// Обрабатывает `M109` — задаёт целевую температуру хотэнда и блокирует
/// исполнение до её достижения.
pub fn handle_set_hotend_temperature_and_wait<C: PrinterContext>(
    context: &mut C,
    command: &GcodeCommand,
) -> AppResult<Option<String>> {
    if let Some(celsius) = command.get('S') {
        context.set_hotend_target(celsius)?;
    }
    context.wait_for_hotend_target()?;
    Ok(None)
}

/// Обрабатывает `M140` — задаёт целевую температуру стола без ожидания.
pub fn handle_set_bed_temperature<C: PrinterContext>(
    context: &mut C,
    command: &GcodeCommand,
) -> AppResult<Option<String>> {
    if let Some(celsius) = command.get('S') {
        context.set_bed_target(celsius)?;
    }
    Ok(None)
}

/// Обрабатывает `M190` — задаёт целевую температуру стола и блокирует
/// исполнение до её достижения.
pub fn handle_set_bed_temperature_and_wait<C: PrinterContext>(
    context: &mut C,
    command: &GcodeCommand,
) -> AppResult<Option<String>> {
    if let Some(celsius) = command.get('S') {
        context.set_bed_target(celsius)?;
    }
    context.wait_for_bed_target()?;
    Ok(None)
}

/// Обрабатывает `M105` — отчёт о текущих и целевых температурах в формате,
/// совместимом по духу с Marlin/RepRap (`T:<hotend> /<target> B:<bed> /<target>`).
pub fn handle_report_temperatures<C: PrinterContext>(context: &C) -> AppResult<Option<String>> {
    Ok(Some(format!(
        "T:{:.1} /{:.1} B:{:.1} /{:.1}",
        context.hotend_temperature(),
        context.hotend_target(),
        context.bed_temperature(),
        context.bed_target(),
    )))
}

/// Обрабатывает `M106` — устанавливает скорость вентилятора обдува детали.
/// Без параметра `S` соответствует полной скорости (стандарт RepRap).
pub fn handle_set_fan_speed<C: PrinterContext>(context: &mut C, command: &GcodeCommand) -> AppResult<Option<String>> {
    let pwm = command
        .get('S')
        .map(|s| s.clamp(0.0, 255.0) as u8)
        .unwrap_or(DEFAULT_FAN_SPEED);
    context.set_part_fan_speed(pwm)?;
    Ok(None)
}

/// Обрабатывает `M107` — полная остановка вентилятора обдува детали.
pub fn handle_fan_off<C: PrinterContext>(context: &mut C) -> AppResult<Option<String>> {
    context.set_part_fan_speed(0)?;
    Ok(None)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gcode::parser::parse_line;
    use std::cell::Cell;

    /// Минимальная фиктивная реализация [`PrinterContext`] для тестов этого
    /// файла — все методы, не относящиеся к температуре/вентилятору,
    /// возвращают заглушки, так как обработчики выше их не вызывают.
    struct StubContext {
        hotend_target: Cell<f32>,
        bed_target: Cell<f32>,
        fan_pwm: Cell<u8>,
    }

    impl Default for StubContext {
        fn default() -> Self {
            Self {
                hotend_target: Cell::new(0.0),
                bed_target: Cell::new(0.0),
                fan_pwm: Cell::new(0),
            }
        }
    }

    impl PrinterContext for StubContext {
        fn plan_linear_move(&mut self, _target: crate::motion::CartesianPosition, _feed_rate_mm_s: f32) -> AppResult<()> {
            Ok(())
        }
        fn current_position(&self) -> crate::motion::CartesianPosition {
            crate::motion::CartesianPosition { x: 0.0, y: 0.0, z: 0.0 }
        }
        fn set_current_position(&mut self, _position: crate::motion::CartesianPosition) {}
        fn home_axes(&mut self, _axes: crate::gcode::commands::AxisSelector) -> AppResult<()> {
            Ok(())
        }
        fn enable_motors(&mut self, _axes: crate::gcode::commands::AxisSelector) -> AppResult<()> {
            Ok(())
        }
        fn disable_motors(&mut self, _axes: crate::gcode::commands::AxisSelector) -> AppResult<()> {
            Ok(())
        }
        fn delay_ms(&mut self, _milliseconds: u32) {}
        fn set_hotend_target(&mut self, celsius: f32) -> AppResult<()> {
            self.hotend_target.set(celsius);
            Ok(())
        }
        fn hotend_temperature(&self) -> f32 {
            self.hotend_target.get()
        }
        fn hotend_target(&self) -> f32 {
            self.hotend_target.get()
        }
        fn wait_for_hotend_target(&mut self) -> AppResult<()> {
            Ok(())
        }
        fn set_bed_target(&mut self, celsius: f32) -> AppResult<()> {
            self.bed_target.set(celsius);
            Ok(())
        }
        fn bed_temperature(&self) -> f32 {
            self.bed_target.get()
        }
        fn bed_target(&self) -> f32 {
            self.bed_target.get()
        }
        fn wait_for_bed_target(&mut self) -> AppResult<()> {
            Ok(())
        }
        fn set_part_fan_speed(&mut self, speed_0_255: u8) -> AppResult<()> {
            self.fan_pwm.set(speed_0_255);
            Ok(())
        }
        fn firmware_info(&self) -> crate::gcode::commands::FirmwareInfo {
            crate::gcode::commands::FirmwareInfo {
                firmware_name: "test",
                firmware_version: "0.0.0",
                kinematics_name: "cartesian",
                extruder_count: 1,
            }
        }
        fn endstop_states(&self) -> AppResult<crate::gcode::commands::EndstopStates> {
            Ok(crate::gcode::commands::EndstopStates {
                x_triggered: false,
                y_triggered: false,
                z_triggered: false,
            })
        }
        fn save_settings(&mut self) -> AppResult<()> {
            Ok(())
        }
        fn load_settings(&mut self) -> AppResult<()> {
            Ok(())
        }
    }

    #[test]
    fn m104_sets_hotend_target_without_waiting() {
        let mut ctx = StubContext::default();
        let cmd = parse_line("M104 S200").unwrap().unwrap();
        handle_set_hotend_temperature(&mut ctx, &cmd).unwrap();
        assert_eq!(ctx.hotend_target.get(), 200.0);
    }

    #[test]
    fn m106_without_param_defaults_to_full_speed() {
        let mut ctx = StubContext::default();
        let cmd = parse_line("M106").unwrap().unwrap();
        handle_set_fan_speed(&mut ctx, &cmd).unwrap();
        assert_eq!(ctx.fan_pwm.get(), 255);
    }

    #[test]
    fn m107_stops_fan() {
        let mut ctx = StubContext::default();
        ctx.fan_pwm.set(200);
        handle_fan_off(&mut ctx).unwrap();
        assert_eq!(ctx.fan_pwm.get(), 0);
    }

    #[test]
    fn m105_report_contains_all_four_values() {
        let mut ctx = StubContext::default();
        ctx.set_hotend_target(210.0).unwrap();
        ctx.set_bed_target(60.0).unwrap();
        let report = handle_report_temperatures(&ctx).unwrap().unwrap();
        assert!(report.contains("T:210.0"));
        assert!(report.contains("B:60.0"));
    }
}
