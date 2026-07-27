//! Исполнитель G-Code: разбирает строку через [`crate::gcode::parser`] и
//! диспетчеризует её в соответствующий обработчик из
//! [`crate::gcode::commands`], поддерживая состояние, которое должно
//! сохраняться между строками (режим позиционирования, последняя скорость
//! подачи — см. [`GcodeState`]).

use crate::error::{AppError, AppResult};
use crate::gcode::commands::motion;
use crate::gcode::commands::system;
use crate::gcode::commands::{temperature, GcodeState, PrinterContext};
use crate::gcode::parser::{self, GcodeCommand};

/// Исполнитель потока строк G-Code поверх произвольной реализации
/// [`PrinterContext`].
///
/// Не содержит собственной логики движения/температуры/хранения — только
/// маршрутизацию команд и состояние интерпретации (`G90`/`G91`, `F`).
pub struct GcodeExecutor<C: PrinterContext> {
    context: C,
    state: GcodeState,
}

impl<C: PrinterContext> GcodeExecutor<C> {
    /// Создаёт исполнитель над готовым контекстом станка.
    #[must_use]
    pub fn new(context: C) -> Self {
        Self {
            context,
            state: GcodeState::default(),
        }
    }

    /// Разбирает и исполняет одну строку G-Code.
    ///
    /// Возвращает:
    /// - `Ok(Some(text))` — команда сформировала ответ (`M105`, `M114`,
    ///   `M115`, `M119`), который должен быть отправлен обратно по каналу,
    ///   из которого пришла команда (USB CDC, WebSocket, ...);
    /// - `Ok(None)` — команда выполнена, ответа не требуется (обычный `ok`
    ///   протокола RepRap формируется вызывающим кодом, а не исполнителем —
    ///   исполнитель не знает, по какому каналу общается);
    /// - `Err(_)` — ошибка разбора или выполнения.
    pub fn execute_line(&mut self, raw_line: &str) -> AppResult<Option<String>> {
        let Some(command) = parser::parse_line(raw_line)? else {
            return Ok(None);
        };
        self.execute_command(&command)
    }

    /// Диспетчеризует уже разобранную команду. Вынесено отдельно от
    /// [`GcodeExecutor::execute_line`], чтобы код, читающий команды из
    /// иного источника, чем построчный текстовый поток (например, очередь
    /// команд с SD-карты, разобранных заранее), мог обойти повторный
    /// парсинг.
    pub fn execute_command(&mut self, command: &GcodeCommand) -> AppResult<Option<String>> {
        match (command.letter, command.code) {
            ('G', 0) | ('G', 1) => motion::handle_linear_move(&mut self.context, &mut self.state, command),
            ('G', 4) => motion::handle_dwell(&mut self.context, command),
            ('G', 28) => motion::handle_home(&mut self.context, command),
            ('G', 90) => motion::handle_absolute_positioning(&mut self.state),
            ('G', 91) => motion::handle_relative_positioning(&mut self.state),
            ('G', 92) => motion::handle_set_position(&mut self.context, command),

            ('M', 17) => motion::handle_enable_motors(&mut self.context, command),
            ('M', 18) => motion::handle_disable_motors(&mut self.context, command),

            ('M', 104) => temperature::handle_set_hotend_temperature(&mut self.context, command),
            ('M', 105) => temperature::handle_report_temperatures(&self.context),
            ('M', 106) => temperature::handle_set_fan_speed(&mut self.context, command),
            ('M', 107) => temperature::handle_fan_off(&mut self.context),
            ('M', 109) => temperature::handle_set_hotend_temperature_and_wait(&mut self.context, command),
            ('M', 140) => temperature::handle_set_bed_temperature(&mut self.context, command),
            ('M', 190) => temperature::handle_set_bed_temperature_and_wait(&mut self.context, command),

            ('M', 114) => system::handle_report_position(&self.context),
            ('M', 115) => system::handle_report_firmware_info(&self.context),
            ('M', 119) => system::handle_report_endstops(&self.context),
            ('M', 500) => system::handle_save_settings(&mut self.context),
            ('M', 501) => system::handle_load_settings(&mut self.context),

            (letter, code) => Err(AppError::GCode {
                line: command.line_number.unwrap_or(0),
                reason: format!("неподдерживаемая команда {letter}{code}"),
            }),
        }
    }

    /// Текущее состояние интерпретации (режим позиционирования, скорость
    /// подачи) — используется диагностикой/тестами.
    #[must_use]
    pub fn state(&self) -> GcodeState {
        self.state
    }

    /// Доступ к контексту станка (например, для получения владения им
    /// обратно после завершения работы исполнителя).
    pub fn into_context(self) -> C {
        self.context
    }

    /// Неизменяемый доступ к контексту станка вне диспетчеризации команд
    /// (например, для сбора телеметрии).
    pub fn context(&self) -> &C {
        &self.context
    }

    /// Изменяемый доступ к контексту станка для действий, не являющихся
    /// обработкой конкретной команды G-Code — например, периодического
    /// обновления регуляторов температуры или прокачки очереди
    /// планировщика движения из главного цикла `App`.
    pub fn context_mut(&mut self) -> &mut C {
        &mut self.context
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gcode::commands::{AxisSelector, EndstopStates, FirmwareInfo, PositioningMode};
    use crate::motion::CartesianPosition;

    /// Фиктивный контекст станка, достаточный для сквозных тестов
    /// исполнителя: движение — упрощённое (без планировщика, немедленно
    /// обновляет позицию), температура/вентилятор/настройки — счётчики.
    #[derive(Default)]
    struct RecordingContext {
        position: CartesianPosition,
        hotend_target: f32,
        bed_target: f32,
        fan_pwm: u8,
        homed: AxisSelector,
        motors_enabled: bool,
    }

    impl PrinterContext for RecordingContext {
        fn plan_linear_move(&mut self, target: CartesianPosition, _feed_rate_mm_s: f32) -> AppResult<()> {
            self.position = target;
            Ok(())
        }
        fn current_position(&self) -> CartesianPosition {
            self.position
        }
        fn set_current_position(&mut self, position: CartesianPosition) {
            self.position = position;
        }
        fn home_axes(&mut self, axes: AxisSelector) -> AppResult<()> {
            self.homed = axes;
            Ok(())
        }
        fn enable_motors(&mut self, _axes: AxisSelector) -> AppResult<()> {
            self.motors_enabled = true;
            Ok(())
        }
        fn disable_motors(&mut self, _axes: AxisSelector) -> AppResult<()> {
            self.motors_enabled = false;
            Ok(())
        }
        fn delay_ms(&mut self, _milliseconds: u32) {}
        fn set_hotend_target(&mut self, celsius: f32) -> AppResult<()> {
            self.hotend_target = celsius;
            Ok(())
        }
        fn hotend_temperature(&self) -> f32 {
            self.hotend_target
        }
        fn hotend_target(&self) -> f32 {
            self.hotend_target
        }
        fn wait_for_hotend_target(&mut self) -> AppResult<()> {
            Ok(())
        }
        fn set_bed_target(&mut self, celsius: f32) -> AppResult<()> {
            self.bed_target = celsius;
            Ok(())
        }
        fn bed_temperature(&self) -> f32 {
            self.bed_target
        }
        fn bed_target(&self) -> f32 {
            self.bed_target
        }
        fn wait_for_bed_target(&mut self) -> AppResult<()> {
            Ok(())
        }
        fn set_part_fan_speed(&mut self, speed_0_255: u8) -> AppResult<()> {
            self.fan_pwm = speed_0_255;
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
    fn g1_moves_and_positioning_mode_persists_across_lines() {
        let mut executor = GcodeExecutor::new(RecordingContext::default());

        executor.execute_line("G91").unwrap();
        assert_eq!(executor.state().positioning_mode, PositioningMode::Relative);

        executor.execute_line("G1 X10 Y5 F1200").unwrap();
        let ctx = executor.into_context();
        assert_eq!((ctx.position.x, ctx.position.y), (10.0, 5.0));
    }

    #[test]
    fn m105_returns_temperature_report() {
        let mut executor = GcodeExecutor::new(RecordingContext::default());
        executor.execute_line("M104 S200").unwrap();
        let response = executor.execute_line("M105").unwrap();
        assert!(response.unwrap().contains("T:200.0"));
    }

    #[test]
    fn unsupported_command_returns_gcode_error() {
        let mut executor = GcodeExecutor::new(RecordingContext::default());
        let result = executor.execute_line("G17");
        assert!(result.is_err());
    }

    #[test]
    fn blank_line_is_silently_ignored() {
        let mut executor = GcodeExecutor::new(RecordingContext::default());
        assert_eq!(executor.execute_line("; comment only").unwrap(), None);
    }

    #[test]
    fn m17_enables_and_m18_disables_motors() {
        let mut executor = GcodeExecutor::new(RecordingContext::default());
        executor.execute_line("M17").unwrap();
        executor.execute_line("M18").unwrap();
        let ctx = executor.into_context();
        assert!(!ctx.motors_enabled);
    }
}
