//! [`PrinterState`] — конкретная реализация [`PrinterContext`], которую
//! видит [`crate::gcode::executor::GcodeExecutor`] при финальной сборке
//! прошивки. Объединяет все уже независимо реализованные и протестированные
//! подсистемы (`motion`, `temperature`, `endstops`, `storage`) под одной
//! границей абстракции — сама по себе не содержит бизнес-логики сверх
//! маршрутизации вызовов и простых процедур (хоуминг, блокирующее ожидание
//! температуры), для которых не нашлось более подходящего места.

use crate::config::AppConfig;
use crate::endstops::EndstopSet;
use crate::error::{AppError, AppResult};
use crate::gcode::commands::{AxisSelector, EndstopStates, FirmwareInfo, PrinterContext};
use crate::hal_adapters::EspLedcPwm;
use crate::motion::{CartesianPosition, EtsStepClock, MotionPlanner, StepGenerator};
use crate::storage::SettingsManager;
use crate::temperature::TemperatureController;
use crate::types::{AxisId, MotorDirection};
use esp_idf_hal::delay::Ets;
use esp_idf_hal::gpio::{AnyIOPin, Input, PinDriver};
use std::cell::RefCell;
use std::time::{Duration, Instant};

/// Версия прошивки (совпадает с версией из `Cargo.toml`).
const FIRMWARE_VERSION: &str = env!("CARGO_PKG_VERSION");
/// Период опроса при блокирующем ожидании температуры.
const WAIT_POLL_PERIOD: Duration = Duration::from_millis(250);
/// Период между шагами при хоуминге (задаёт медленную, безопасную скорость
/// поиска концевика — существенно ниже рабочих скоростей печати).
const HOMING_STEP_DELAY_US: u32 = 1500;
/// Запас хода при хоуминге сверх номинальной длины оси, доля от неё —
/// защита от бесконечного движения при неисправном концевике.
const HOMING_OVERTRAVEL_FACTOR: f32 = 1.2;

type EndstopInputPin = PinDriver<'static, AnyIOPin, Input>;
type PrinterEndstops = EndstopSet<EndstopInputPin, EndstopInputPin, EndstopInputPin>;
type PrinterTemperature = TemperatureController<
    crate::hal_adapters::EspAdcThermistor,
    EspLedcPwm<'static>,
    crate::hal_adapters::EspAdcThermistor,
    EspLedcPwm<'static>,
    EspLedcPwm<'static>,
>;

/// Конкретная реализация [`PrinterContext`] для станка OSIX.
pub struct PrinterState {
    pub(crate) planner: MotionPlanner,
    pub(crate) step_generator: StepGenerator<EtsStepClock>,
    pub(crate) temperature: PrinterTemperature,
    endstops: RefCell<PrinterEndstops>,
    settings: SettingsManager,
    config: AppConfig,
    kinematics_name: &'static str,
    start_instant: Instant,
}

impl PrinterState {
    /// Собирает состояние станка из уже полностью сконфигурированных
    /// подсистем (см. `app::build_printer_state`, где конкретные типы
    /// драйверов/ADC/ШИМ создаются из пинов `Board`).
    #[must_use]
    pub fn new(
        planner: MotionPlanner,
        step_generator: StepGenerator<EtsStepClock>,
        temperature: PrinterTemperature,
        endstops: PrinterEndstops,
        settings: SettingsManager,
        config: AppConfig,
        kinematics_name: &'static str,
    ) -> Self {
        Self {
            planner,
            step_generator,
            temperature,
            endstops: RefCell::new(endstops),
            settings,
            config,
            kinematics_name,
            start_instant: Instant::now(),
        }
    }

    /// Время, прошедшее с создания состояния станка, секунды — используется
    /// для окон наблюдения thermal runaway и телеметрии.
    fn elapsed_seconds(&self) -> f64 {
        self.start_instant.elapsed().as_secs_f64()
    }

    /// Один такт регулирования температуры — должен вызываться
    /// периодически из главного цикла [`crate::app::App::run`], не как
    /// часть обработки конкретной команды G-Code.
    pub fn tick_temperature(&mut self, dt_seconds: f32) -> AppResult<()> {
        let time_s = self.elapsed_seconds();
        self.temperature.update(dt_seconds, time_s)
    }

    /// Извлекает из очереди планировщика и исполняет один готовый сегмент
    /// движения, если он есть. Возвращает `true`, если сегмент был исполнен.
    ///
    /// Должен вызываться из главного цикла достаточно часто, чтобы очередь
    /// не переполнялась быстрее, чем печатаются сегменты — при
    /// однопоточной кооперативной схеме этого коммита один сегмент
    /// исполняется полностью (блокируя цикл на его физическую
    /// длительность) перед обработкой следующей строки G-Code.
    pub fn pump_motion(&mut self) -> AppResult<bool> {
        match self.planner.pop_next_segment() {
            Some(segment) => {
                self.step_generator.execute_segment(&segment)?;
                Ok(true)
            }
            None => Ok(false),
        }
    }

    /// Отображает идентификатор оси на индекс в массиве `StepGenerator`
    /// (`X`→`0`, `Y`→`1`, `Z`→`2` — то же соответствие, что используется
    /// во всём проекте, см. `motion::kinematics`).
    fn axis_index(axis: AxisId) -> usize {
        match axis {
            AxisId::X => 0,
            AxisId::Y => 1,
            AxisId::Z => 2,
        }
    }

    /// Выполняет хоуминг одной оси: движение к концевику на медленной
    /// постоянной скорости с ограничением по числу шагов (защита от
    /// бесконечного движения при неисправном концевике).
    fn home_single_axis(&mut self, axis: AxisId) -> AppResult<()> {
        let axis_config = self
            .config
            .motion
            .axis(axis)
            .ok_or_else(|| AppError::config("motion.toml", format!("отсутствует конфигурация оси {axis}")))?;

        let max_steps = (axis_config.max_position_mm.max(1.0)
            * axis_config.steps_per_mm
            * HOMING_OVERTRAVEL_FACTOR) as u64;

        let index = Self::axis_index(axis);
        {
            let axis_control = &mut self.step_generator.axes_mut()[index];
            axis_control.enable()?;
            axis_control.set_direction(MotorDirection::Backward)?;
        }

        let mut steps_taken = 0u64;
        loop {
            if self.endstops.borrow_mut().is_axis_triggered(axis)? {
                break;
            }
            if steps_taken >= max_steps {
                return Err(AppError::HardwareTimeout(format!(
                    "хоуминг оси {axis} не обнаружил концевик за {max_steps} шагов"
                )));
            }

            self.step_generator.axes_mut()[index].step()?;
            Ets::delay_us(HOMING_STEP_DELAY_US);
            steps_taken += 1;
        }

        self.step_generator.axes_mut()[index].reset_position(0);
        log::info!("ось {axis} захоумлена за {steps_taken} шагов");
        Ok(())
    }
}

impl PrinterContext for PrinterState {
    fn plan_linear_move(&mut self, target: CartesianPosition, feed_rate_mm_s: f32) -> AppResult<()> {
        // Планировщик может отклонить постановку в очередь, если она
        // заполнена — в этом случае вытесняем накопившиеся сегменты
        // движением вперёд, прежде чем повторить попытку, не блокируя
        // исполнителя G-Code бесконечно.
        loop {
            if self.planner.plan_linear_move(target, feed_rate_mm_s)? {
                return Ok(());
            }
            self.pump_motion()?;
        }
    }

    fn current_position(&self) -> CartesianPosition {
        self.planner.current_position()
    }

    fn set_current_position(&mut self, position: CartesianPosition) {
        self.planner.set_current_position(position);
    }

    fn home_axes(&mut self, axes: AxisSelector) -> AppResult<()> {
        if axes.x {
            self.home_single_axis(AxisId::X)?;
        }
        if axes.y {
            self.home_single_axis(AxisId::Y)?;
        }
        if axes.z {
            self.home_single_axis(AxisId::Z)?;
        }
        Ok(())
    }

    fn enable_motors(&mut self, axes: AxisSelector) -> AppResult<()> {
        if axes.x {
            self.step_generator.axes_mut()[Self::axis_index(AxisId::X)].enable()?;
        }
        if axes.y {
            self.step_generator.axes_mut()[Self::axis_index(AxisId::Y)].enable()?;
        }
        if axes.z {
            self.step_generator.axes_mut()[Self::axis_index(AxisId::Z)].enable()?;
        }
        Ok(())
    }

    fn disable_motors(&mut self, axes: AxisSelector) -> AppResult<()> {
        if axes.x {
            self.step_generator.axes_mut()[Self::axis_index(AxisId::X)].disable()?;
        }
        if axes.y {
            self.step_generator.axes_mut()[Self::axis_index(AxisId::Y)].disable()?;
        }
        if axes.z {
            self.step_generator.axes_mut()[Self::axis_index(AxisId::Z)].disable()?;
        }
        Ok(())
    }

    fn delay_ms(&mut self, milliseconds: u32) {
        std::thread::sleep(Duration::from_millis(u64::from(milliseconds)));
    }

    fn set_hotend_target(&mut self, celsius: f32) -> AppResult<()> {
        self.temperature.set_hotend_target(celsius)
    }

    fn hotend_temperature(&self) -> f32 {
        self.temperature.hotend_temperature()
    }

    fn hotend_target(&self) -> f32 {
        self.temperature.hotend_target()
    }

    fn wait_for_hotend_target(&mut self) -> AppResult<()> {
        while !self.temperature.is_hotend_at_target() {
            self.tick_temperature(WAIT_POLL_PERIOD.as_secs_f32())?;
            std::thread::sleep(WAIT_POLL_PERIOD);
        }
        Ok(())
    }

    fn set_bed_target(&mut self, celsius: f32) -> AppResult<()> {
        self.temperature.set_bed_target(celsius)
    }

    fn bed_temperature(&self) -> f32 {
        self.temperature.bed_temperature()
    }

    fn bed_target(&self) -> f32 {
        self.temperature.bed_target()
    }

    fn wait_for_bed_target(&mut self) -> AppResult<()> {
        while !self.temperature.is_bed_at_target() {
            self.tick_temperature(WAIT_POLL_PERIOD.as_secs_f32())?;
            std::thread::sleep(WAIT_POLL_PERIOD);
        }
        Ok(())
    }

    fn set_part_fan_speed(&mut self, speed_0_255: u8) -> AppResult<()> {
        self.temperature.set_part_fan_speed(speed_0_255)
    }

    fn firmware_info(&self) -> FirmwareInfo {
        FirmwareInfo {
            firmware_name: "OSIX Firmware",
            firmware_version: FIRMWARE_VERSION,
            kinematics_name: self.kinematics_name,
            extruder_count: self.config.printer.extruder_count,
        }
    }

    fn endstop_states(&self) -> AppResult<EndstopStates> {
        self.endstops.borrow_mut().states()
    }

    fn save_settings(&mut self) -> AppResult<()> {
        self.settings.save(&self.config)
    }

    fn load_settings(&mut self) -> AppResult<()> {
        self.config = self.settings.load()?;
        log::warn!(
            "настройки загружены с флеш-памяти; для применения параметров движения и температуры может потребоваться перезагрузка станка"
        );
        Ok(())
    }
}
