//! Сборка конкретного железа станка из пинов [`crate::board::Board`] в
//! готовый [`crate::printer_state::PrinterState`].
//!
//! Вынесено из [`crate::app`] в отдельный файл, чтобы держать создание
//! `App` (жизненный цикл, главный цикл) отдельно от проводки конкретных
//! пинов/ADC/ШИМ в абстрактные подсистемы (`drivers::motor`, `temperature`,
//! `endstops`) — то единственное место во всём проекте, где эти
//! абстракции встречаются с конкретным железом ESP32-S3.

use crate::board::pins::PinMap;
use crate::config::AppConfig;
use crate::drivers::motor::axis::{Axis, AxisControl};
use crate::drivers::motor::tmc2209::{CurrentSenseConfig, MicrostepResolution, Tmc2209Driver, Tmc2209Uart};
use crate::drivers::motor::uln2003::{StepMode, Uln2003Driver, Uln2003Pins};
use crate::endstops::{Endstop, EndstopSet};
use crate::error::{AppError, AppResult};
use crate::hal_adapters::{EspAdcThermistor, EspLedcPwm};
use crate::motion::kinematics::{CartesianKinematics, CoreXyKinematics, CoreXzKinematics, DeltaGeometry, DeltaKinematics, Kinematics};
use crate::motion::{EtsStepClock, MotionPlanner, StepGenerator};
use crate::printer_state::PrinterState;
use crate::storage::SettingsManager;
use crate::temperature::fan::Fan;
use crate::temperature::heater::Heater;
use crate::temperature::thermistor::Thermistor;
use crate::temperature::TemperatureController;
use crate::types::{AxisId, Milliamps};
use esp_idf_hal::gpio::{AnyIOPin, AnyOutputPin, Input, Output, PinDriver, Pull};
use esp_idf_hal::ledc::{config::TimerConfig, LedcDriver, LedcTimerDriver};
use esp_idf_hal::uart::{config::Config as UartConfig, UartDriver};
use esp_idf_hal::units::Hertz;

/// Ток движения TMC2209 по умолчанию (NEMA17 общего назначения, ~1.5А
/// заявленный ток обмотки — 800 мА RMS оставляет комфортный запас без
/// перегрева на большинстве плат радиаторов TMC2209).
const DEFAULT_RUN_CURRENT_MA: u16 = 800;
/// Ток удержания TMC2209 по умолчанию — существенно ниже рабочего, чтобы
/// не перегревать двигатель/драйвер в состоянии покоя.
const DEFAULT_HOLD_CURRENT_MA: u16 = 300;
/// Частота ШИМ нагревателей/вентилятора, Гц — ниже слышимого писка
/// дешёвых MOSFET-модулей, но достаточно высокая для гладкого
/// регулирования мощности нагревателя.
const HEATER_PWM_FREQUENCY_HZ: u32 = 1000;

/// Собирает [`PrinterState`] целиком: драйверы моторов, кинематику,
/// планировщик движения, генератор шагов, контуры температуры, концевики
/// и менеджер настроек.
///
/// Принимает уже распакованные части [`Board`] (а не саму `Board`
/// целиком), поскольку `power`/`watchdog`/`rgb` остаются непосредственно у
/// [`crate::app::App`] для главного цикла — `Board::init()` вызывается
/// ровно один раз за всё время работы программы (повторный вызов
/// `Peripherals::take()` внутри нее завершился бы ошибкой), поэтому
/// саму `Board` нельзя разобрать в двух местах по отдельности.
pub fn build_printer_state(
    pins: PinMap,
    uart1: esp_idf_hal::uart::UART1,
    uart2: esp_idf_hal::uart::UART2,
    ledc: esp_idf_hal::ledc::LEDC,
    config: AppConfig,
) -> AppResult<PrinterState> {
    let kinematics = build_kinematics(&config);
    let kinematics_name = kinematics.name();

    let axes = build_axes(&pins, uart1, uart2)?;
    let steps_per_mm = [
        config.motion.axis(AxisId::X).map(|a| a.steps_per_mm).unwrap_or(80.0),
        config.motion.axis(AxisId::Y).map(|a| a.steps_per_mm).unwrap_or(80.0),
        config.motion.axis(AxisId::Z).map(|a| a.steps_per_mm).unwrap_or(80.0),
    ];
    let step_generator = StepGenerator::new(axes, steps_per_mm, config.motion.max_step_rate_hz, EtsStepClock);

    let planner = MotionPlanner::new(&config.motion, kinematics)?;

    let temperature = build_temperature_controller(&pins, &config, ledc)?;
    let endstops = build_endstop_set(&pins)?;
    let settings = SettingsManager::mount()?;

    Ok(PrinterState::new(
        planner,
        step_generator,
        temperature,
        endstops,
        settings,
        config,
        kinematics_name,
    ))
}

/// Выбирает реализацию кинематики согласно `printer.toml`.
fn build_kinematics(config: &AppConfig) -> Box<dyn Kinematics> {
    use crate::config::printer::KinematicsKind;
    match config.printer.kinematics {
        KinematicsKind::Cartesian => Box::new(CartesianKinematics),
        KinematicsKind::CoreXY => Box::new(CoreXyKinematics),
        KinematicsKind::CoreXZ => Box::new(CoreXzKinematics),
        KinematicsKind::Delta => Box::new(DeltaKinematics::new(DeltaGeometry::default())),
    }
}

/// Создаёт выходной пин по номеру GPIO из [`PinMap`].
///
/// # Safety
/// См. обоснование в `board::mod` — единообразный для всего проекта
/// способ получения пинов по номеру в обход системы владения
/// `Peripherals`, применяемый только для GPIO (электрически не
/// эксклюзивных при повторном создании для чтения/независимой настройки).
fn output_pin(gpio: u8) -> AppResult<PinDriver<'static, AnyOutputPin, Output>> {
    let raw = unsafe { AnyOutputPin::new(i32::from(gpio)) };
    PinDriver::output(raw).map_err(|e| AppError::board(format!("не удалось настроить выход GPIO{gpio}: {e}")))
}

/// Создаёт входной пин по номеру GPIO с подтяжкой к питанию (используется
/// концевиками — нормально-замкнутые микровыключатели на землю).
fn pulled_up_input_pin(gpio: u8) -> AppResult<PinDriver<'static, AnyIOPin, Input>> {
    let raw = unsafe { AnyIOPin::new(i32::from(gpio)) };
    let mut driver =
        PinDriver::input(raw).map_err(|e| AppError::board(format!("не удалось настроить вход GPIO{gpio}: {e}")))?;
    driver
        .set_pull(Pull::Up)
        .map_err(|e| AppError::board(format!("не удалось настроить подтяжку GPIO{gpio}: {e}")))?;
    Ok(driver)
}

/// Создаёт три оси (`X`, `Y` — TMC2209 по UART; `Z` — ULN2003) в виде
/// объектно-безопасных [`AxisControl`], готовых для [`StepGenerator`].
///
/// Принимает `uart1`/`uart2` во владение — периферия однократно
/// зарезервирована в `Board::init` и не может быть получена повторно.
fn build_axes(
    pins: &PinMap,
    uart1: esp_idf_hal::uart::UART1,
    uart2: esp_idf_hal::uart::UART2,
) -> AppResult<[Box<dyn AxisControl>; 3]> {
    let uart_config = UartConfig::new().baudrate(Hertz(115_200));

    // --- Ось X: TMC2209 по UART1 -----------------------------------
    let x_tx = unsafe { AnyOutputPin::new(i32::from(pins.tmc_uart.x_tx)) };
    let x_rx = unsafe { AnyIOPin::new(i32::from(pins.tmc_uart.x_rx)) };
    let x_uart_driver = UartDriver::new(
        uart1,
        x_tx,
        x_rx,
        Option::<AnyIOPin>::None,
        Option::<AnyIOPin>::None,
        &uart_config,
    )
    .map_err(|e| AppError::board(format!("не удалось создать UART для TMC2209 X: {e}")))?;

    let mut x_driver = Tmc2209Driver::init(
        Tmc2209Uart::new(x_uart_driver, 0),
        output_pin(pins.axis_x.step)?,
        output_pin(pins.axis_x.dir)?,
        output_pin(pins.axis_x.enable)?,
        CurrentSenseConfig::default(),
    )?;
    x_driver.set_microsteps(MicrostepResolution::Full16)?;
    x_driver.set_current(Milliamps(DEFAULT_RUN_CURRENT_MA), Milliamps(DEFAULT_HOLD_CURRENT_MA), 4)?;
    x_driver.enable_stealth_chop()?;
    let x_axis = Axis::new(AxisId::X, x_driver, pulled_up_input_pin(pins.axis_x.endstop)?, false, true);

    // --- Ось Y: TMC2209 по UART2 -----------------------------------
    let y_tx = unsafe { AnyOutputPin::new(i32::from(pins.tmc_uart.y_tx)) };
    let y_rx = unsafe { AnyIOPin::new(i32::from(pins.tmc_uart.y_rx)) };
    let y_uart_driver = UartDriver::new(
        uart2,
        y_tx,
        y_rx,
        Option::<AnyIOPin>::None,
        Option::<AnyIOPin>::None,
        &uart_config,
    )
    .map_err(|e| AppError::board(format!("не удалось создать UART для TMC2209 Y: {e}")))?;

    let mut y_driver = Tmc2209Driver::init(
        Tmc2209Uart::new(y_uart_driver, 0),
        output_pin(pins.axis_y.step)?,
        output_pin(pins.axis_y.dir)?,
        output_pin(pins.axis_y.enable)?,
        CurrentSenseConfig::default(),
    )?;
    y_driver.set_microsteps(MicrostepResolution::Full16)?;
    y_driver.set_current(Milliamps(DEFAULT_RUN_CURRENT_MA), Milliamps(DEFAULT_HOLD_CURRENT_MA), 4)?;
    y_driver.enable_stealth_chop()?;
    let y_axis = Axis::new(AxisId::Y, y_driver, pulled_up_input_pin(pins.axis_y.endstop)?, false, true);

    // --- Ось Z: ULN2003 (28BYJ-48) -----------------------------------
    let z_pins = Uln2003Pins::new(
        output_pin(pins.axis_z.in1)?,
        output_pin(pins.axis_z.in2)?,
        output_pin(pins.axis_z.in3)?,
        output_pin(pins.axis_z.in4)?,
    );
    let z_driver = Uln2003Driver::new(z_pins, StepMode::Half)?;
    let z_axis = Axis::new(AxisId::Z, z_driver, pulled_up_input_pin(pins.axis_z.endstop)?, false, true);

    Ok([
        Box::new(x_axis) as Box<dyn AxisControl>,
        Box::new(y_axis) as Box<dyn AxisControl>,
        Box::new(z_axis) as Box<dyn AxisControl>,
    ])
}

/// Создаёт контроллер температуры (хотэнд + стол + вентилятор) поверх
/// ADC1 (термисторы) и LEDC (ШИМ нагревателей/вентилятора).
///
/// Принимает `ledc` (группу таймеров/каналов LEDC) во владение — периферия
/// однократно зарезервирована в `Board::init` и не может быть получена
/// повторно.
///
/// Примечание для проверки при первой сборке: предполагается, что
/// `esp_idf_hal::ledc::LEDC` содержит публичные поля `timer0`, `channel0`,
/// `channel1`, `channel2` (по аналогии с `uart1`/`spi2` в `Peripherals`,
/// уже успешно используемыми в `Board`) — это устоявшийся, но не
/// абсолютно неизменный аспект структуры периферии `esp-idf-hal`.
fn build_temperature_controller(
    pins: &PinMap,
    config: &AppConfig,
    ledc: esp_idf_hal::ledc::LEDC,
) -> AppResult<
    TemperatureController<EspAdcThermistor, EspLedcPwm<'static>, EspAdcThermistor, EspLedcPwm<'static>, EspLedcPwm<'static>>,
> {
    // Таймер LEDC разделяется тремя каналами (хотэнд/стол/вентилятор) и
    // должен жить не меньше, чем сами каналы. Поскольку прошивка работает
    // всё время жизни программы и никогда не "освобождает" этот таймер,
    // сознательно продлеваем его время жизни до `'static` через
    // `Box::leak` — стандартный приём для синглтон-ресурсов встраиваемых
    // систем, а не утечка в обычном смысле (память не может быть
    // переиспользована в любом случае, пока включён контроллер).
    let timer_config = TimerConfig::new().frequency(Hertz(HEATER_PWM_FREQUENCY_HZ));
    let timer: &'static LedcTimerDriver<'static> = Box::leak(Box::new(
        LedcTimerDriver::new(ledc.timer0, &timer_config)
            .map_err(|e| AppError::board(format!("не удалось настроить таймер LEDC: {e}")))?,
    ));

    let hotend_pwm_pin = unsafe { AnyOutputPin::new(i32::from(pins.temperature.hotend_heater_pwm)) };
    let hotend_channel = LedcDriver::new(ledc.channel0, timer, hotend_pwm_pin)
        .map_err(|e| AppError::board(format!("не удалось создать канал ШИМ хотэнда: {e}")))?;

    let bed_pwm_pin = unsafe { AnyOutputPin::new(i32::from(pins.temperature.bed_heater_pwm)) };
    let bed_channel = LedcDriver::new(ledc.channel1, timer, bed_pwm_pin)
        .map_err(|e| AppError::board(format!("не удалось создать канал ШИМ стола: {e}")))?;

    let fan_pwm_pin = unsafe { AnyOutputPin::new(i32::from(pins.temperature.part_fan_pwm)) };
    let fan_channel = LedcDriver::new(ledc.channel2, timer, fan_pwm_pin)
        .map_err(|e| AppError::board(format!("не удалось создать канал ШИМ вентилятора: {e}")))?;

    let hotend_thermistor = Thermistor::new(
        EspAdcThermistor::new(pins.temperature.hotend_thermistor_adc_channel)?,
        config.temperature.hotend.thermistor,
    );
    let bed_thermistor = Thermistor::new(
        EspAdcThermistor::new(pins.temperature.bed_thermistor_adc_channel)?,
        config.temperature.bed.thermistor,
    );

    let hotend = Heater::new(hotend_thermistor, EspLedcPwm::new(hotend_channel), config.temperature.hotend);
    let bed = Heater::new(bed_thermistor, EspLedcPwm::new(bed_channel), config.temperature.bed);
    let fan = Fan::new(EspLedcPwm::new(fan_channel))?;

    Ok(TemperatureController::new(hotend, bed, fan))
}

/// Создаёт независимый групповой опрос концевиков для `M119`/хоуминга
/// (см. `endstops` — не пересекается по владению с концевиками,
/// встроенными в оси, поскольку чтение GPIO электрически не эксклюзивно).
fn build_endstop_set(
    pins: &PinMap,
) -> AppResult<EndstopSet<PinDriver<'static, AnyIOPin, Input>, PinDriver<'static, AnyIOPin, Input>, PinDriver<'static, AnyIOPin, Input>>>
{
    Ok(EndstopSet::new(
        Endstop::new(pulled_up_input_pin(pins.axis_x.endstop)?, true),
        Endstop::new(pulled_up_input_pin(pins.axis_y.endstop)?, true),
        Endstop::new(pulled_up_input_pin(pins.axis_z.endstop)?, true),
    ))
}
