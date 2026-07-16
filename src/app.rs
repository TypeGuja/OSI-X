//! Главная структура приложения `App`.
//!
//! Собирает воедино все подсистемы прошивки через
//! [`crate::hardware_build::build_printer_state`] и запускает кооперативный
//! главный цикл: приём G-Code по USB, прокачка очереди планировщика
//! движения, периодическое регулирование температуры, опрос аварийной
//! остановки и обслуживание watchdog.
//!
//! # Об архитектуре главного цикла
//!
//! Этот цикл — однопоточный и кооперативный: он не использует отдельные
//! задачи FreeRTOS для генератора шагов/G-Code/сети, хотя все необходимые
//! для этого примитивы уже написаны и протестированы
//! (`scheduler::Task`/`TaskPriority`, `network::*`, `sdcard::PrintJob`).
//! Это сознательный выбор ради предсказуемости и проверяемости первой
//! работающей версии: каждый вызов `execute_segment` блокирует цикл на
//! физическую длительность сегмента, поэтому новая строка G-Code не
//! обрабатывается, пока станок находится в движении — компромисс,
//! приемлемый для первой интеграции и устранимый впоследствии переносом
//! `pump_motion`/приёма USB на отдельные задачи `scheduler::Task` с
//! приоритетами `TaskPriority::StepGenerator`/`TaskPriority::GCode` без
//! изменения `PrinterState`/`GcodeExecutor`.

use crate::board::power::Power;
use crate::board::rgb::{Color, RgbStatus};
use crate::board::watchdog::Watchdog;
use crate::board::Board;
use crate::config::AppConfig;
use crate::error::AppResult;
use crate::gcode::executor::GcodeExecutor;
use crate::hardware_build::build_printer_state;
use crate::logger;
use crate::printer_state::PrinterState;
use crate::system;
use crate::usb::{SerialConsole, UsbCdc};
use std::time::{Duration, Instant};

/// Период главного цикла, когда нет ни новой строки G-Code, ни сегмента
/// движения для исполнения — короткий, чтобы не задерживать обслуживание
/// watchdog/E-Stop, но не настолько частый, чтобы впустую грузить CPU.
const IDLE_LOOP_PERIOD: Duration = Duration::from_millis(2);

/// Главная структура приложения OSIX Firmware.
pub struct App {
    power: Power<'static>,
    watchdog: Watchdog,
    rgb: RgbStatus<'static>,
    console: SerialConsole<UsbCdc>,
    executor: GcodeExecutor<PrinterState>,
    temperature_sample_period: Duration,
    last_temperature_tick: Instant,
}

impl App {
    /// Создаёт приложение: инициализирует логирование, диагностику, плату
    /// и все подсистемы станка, затем USB-консоль.
    ///
    /// `Board::init()` вызывается ровно один раз (захватывает
    /// `Peripherals` — повторный захват невозможен), после чего сразу
    /// разбирается на части: `power`/`watchdog`/`rgb` остаются у `App`
    /// напрямую для главного цикла, а пины и периферия (UART/LEDC)
    /// уходят в [`build_printer_state`], где встречаются с конкретными
    /// драйверами моторов/термисторов/нагревателей.
    pub fn new() -> AppResult<Self> {
        let config = AppConfig::default();
        logger::init(logger::level_from_str("info"))?;
        system::init();

        log::info!(
            "запуск OSIX Firmware — станок '{}', кинематика {:?}",
            config.printer.name,
            config.printer.kinematics
        );

        let board: Board<'static> = Board::init()?;
        let Board { power, watchdog, rgb, pins, uart1, uart2, ledc, .. } = board;

        let temperature_sample_period = Duration::from_millis(u64::from(config.temperature.sample_period_ms));

        let printer_state = build_printer_state(pins, uart1, uart2, ledc, config)?;
        let executor = GcodeExecutor::new(printer_state);

        let usb = UsbCdc::install()?;
        let console = SerialConsole::new(usb);

        Ok(Self {
            power,
            watchdog,
            rgb,
            console,
            executor,
            temperature_sample_period,
            last_temperature_tick: Instant::now(),
        })
    }

    /// Запускает главный цикл приложения. В штатной эксплуатации не
    /// завершается.
    pub fn run(&mut self) -> AppResult<()> {
        self.rgb.set_color(Color::READY)?;
        log::info!("инициализация завершена, станок готов к приёму команд по USB");

        loop {
            self.watchdog.reset()?;
            self.power.poll_emergency_stop()?;

            if self.power.is_emergency_stopped() {
                self.rgb.set_color(Color::ERROR)?;
            }

            let mut did_work = false;

            if let Some(line) = self.console.poll_line()? {
                did_work = true;
                self.handle_gcode_line(&line)?;
            }

            if self.executor.context_mut().pump_motion()? {
                did_work = true;
            }

            if self.last_temperature_tick.elapsed() >= self.temperature_sample_period {
                let dt = self.last_temperature_tick.elapsed().as_secs_f32();
                self.executor.context_mut().tick_temperature(dt)?;
                self.last_temperature_tick = Instant::now();
                did_work = true;
            }

            if !did_work {
                std::thread::sleep(IDLE_LOOP_PERIOD);
            }
        }
    }

    /// Обрабатывает одну строку, полученную по USB: исполняет её и
    /// отправляет ответ в формате, ожидаемом хостовыми программами
    /// (отчёт, если команда его подразумевает, затем `ok`; либо `Error: ...`).
    fn handle_gcode_line(&mut self, line: &str) -> AppResult<()> {
        match self.executor.execute_line(line) {
            Ok(Some(report)) => {
                self.console.send_line(&report)?;
                self.console.send_ok()
            }
            Ok(None) => self.console.send_ok(),
            Err(e) => self.console.send_error(&e.to_string()),
        }
    }
}
