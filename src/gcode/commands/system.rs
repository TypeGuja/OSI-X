//! Обработчики системных команд отчёта и настроек: `M114`, `M115`, `M119`,
//! `M500`, `M501`.

use crate::error::AppResult;
use crate::gcode::commands::PrinterContext;

/// Обрабатывает `M114` — отчёт о текущей позиции эффектора.
///
/// Ось `E` (экструдер) всегда отображается как `0.00`: текущая аппаратная
/// конфигурация станка (см. ТЗ, раздел "Первоначальная конфигурация
/// принтера") не включает двигатель экструдера — только X/Y/Z. Параметр
/// сохранён в отчёте для совместимости с хостовым ПО (OctoPrint и т.п.),
/// которое ожидает поле `E` в ответе на `M114`.
pub fn handle_report_position<C: PrinterContext>(context: &C) -> AppResult<Option<String>> {
    let position = context.current_position();
    Ok(Some(format!(
        "X:{:.2} Y:{:.2} Z:{:.2} E:0.00",
        position.x, position.y, position.z
    )))
}

/// Обрабатывает `M115` — отчёт о версии и возможностях прошивки.
pub fn handle_report_firmware_info<C: PrinterContext>(context: &C) -> AppResult<Option<String>> {
    Ok(Some(context.firmware_info().to_report_string()))
}

/// Обрабатывает `M119` — отчёт о состоянии концевых выключателей.
pub fn handle_report_endstops<C: PrinterContext>(context: &C) -> AppResult<Option<String>> {
    Ok(Some(context.endstop_states()?.to_report_string()))
}

/// Обрабатывает `M500` — сохраняет текущие настройки в энергонезависимую
/// память.
pub fn handle_save_settings<C: PrinterContext>(context: &mut C) -> AppResult<Option<String>> {
    context.save_settings()?;
    Ok(None)
}

/// Обрабатывает `M501` — загружает настройки из энергонезависимой памяти,
/// заменяя текущую конфигурацию в оперативной памяти.
pub fn handle_load_settings<C: PrinterContext>(context: &mut C) -> AppResult<Option<String>> {
    context.load_settings()?;
    Ok(None)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gcode::commands::{AxisSelector, EndstopStates, FirmwareInfo};
    use crate::motion::CartesianPosition;
    use std::cell::Cell;

    struct StubContext {
        position: CartesianPosition,
        save_called: Cell<bool>,
        load_called: Cell<bool>,
    }

    impl PrinterContext for StubContext {
        fn plan_linear_move(&mut self, _target: CartesianPosition, _feed_rate_mm_s: f32) -> AppResult<()> {
            Ok(())
        }
        fn current_position(&self) -> CartesianPosition {
            self.position
        }
        fn set_current_position(&mut self, position: CartesianPosition) {
            self.position = position;
        }
        fn home_axes(&mut self, _axes: AxisSelector) -> AppResult<()> {
            Ok(())
        }
        fn enable_motors(&mut self, _axes: AxisSelector) -> AppResult<()> {
            Ok(())
        }
        fn disable_motors(&mut self, _axes: AxisSelector) -> AppResult<()> {
            Ok(())
        }
        fn delay_ms(&mut self, _milliseconds: u32) {}
        fn set_hotend_target(&mut self, _celsius: f32) -> AppResult<()> {
            Ok(())
        }
        fn hotend_temperature(&self) -> f32 {
            0.0
        }
        fn hotend_target(&self) -> f32 {
            0.0
        }
        fn wait_for_hotend_target(&mut self) -> AppResult<()> {
            Ok(())
        }
        fn set_bed_target(&mut self, _celsius: f32) -> AppResult<()> {
            Ok(())
        }
        fn bed_temperature(&self) -> f32 {
            0.0
        }
        fn bed_target(&self) -> f32 {
            0.0
        }
        fn wait_for_bed_target(&mut self) -> AppResult<()> {
            Ok(())
        }
        fn set_part_fan_speed(&mut self, _speed_0_255: u8) -> AppResult<()> {
            Ok(())
        }
        fn firmware_info(&self) -> FirmwareInfo {
            FirmwareInfo {
                firmware_name: "OSIX",
                firmware_version: "0.1.0",
                kinematics_name: "cartesian",
                extruder_count: 1,
            }
        }
        fn endstop_states(&self) -> AppResult<EndstopStates> {
            Ok(EndstopStates {
                x_triggered: true,
                y_triggered: false,
                z_triggered: false,
            })
        }
        fn save_settings(&mut self) -> AppResult<()> {
            self.save_called.set(true);
            Ok(())
        }
        fn load_settings(&mut self) -> AppResult<()> {
            self.load_called.set(true);
            Ok(())
        }
    }

    fn stub() -> StubContext {
        StubContext {
            position: CartesianPosition { x: 1.0, y: 2.0, z: 3.0 },
            save_called: Cell::new(false),
            load_called: Cell::new(false),
        }
    }

    #[test]
    fn m114_reports_current_position_with_zero_extruder() {
        let ctx = stub();
        let report = handle_report_position(&ctx).unwrap().unwrap();
        assert_eq!(report, "X:1.00 Y:2.00 Z:3.00 E:0.00");
    }

    #[test]
    fn m115_reports_firmware_info() {
        let ctx = stub();
        let report = handle_report_firmware_info(&ctx).unwrap().unwrap();
        assert!(report.contains("FIRMWARE_NAME:OSIX"));
    }

    #[test]
    fn m119_reports_endstop_states() {
        let ctx = stub();
        let report = handle_report_endstops(&ctx).unwrap().unwrap();
        assert!(report.contains("x_min: TRIGGERED"));
        assert!(report.contains("y_min: open"));
    }

    #[test]
    fn m500_and_m501_delegate_to_context() {
        let mut ctx = stub();
        handle_save_settings(&mut ctx).unwrap();
        handle_load_settings(&mut ctx).unwrap();
        assert!(ctx.save_called.get());
        assert!(ctx.load_called.get());
    }
}
