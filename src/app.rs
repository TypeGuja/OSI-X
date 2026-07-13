//! Главная структура приложения `App`.
//!
//! `App` отвечает за порядок инициализации подсистем и за верхнеуровневый
//! рабочий цикл. На этапе 1 цикл ограничен опросом аварийной остановки,
//! обслуживанием watchdog и индикацией статуса — очередь движения, G-Code
//! исполнитель и сетевые сервисы подключаются на последующих этапах как
//! дополнительные поля `App` и шаги внутри `run()`, без изменения уже
//! написанного кода инициализации.

use crate::board::rgb::Color;
use crate::board::Board;
use crate::config::AppConfig;
use crate::error::AppResult;
use crate::logger;
use std::time::Duration;

/// Период основного цикла приложения.
const MAIN_LOOP_PERIOD: Duration = Duration::from_millis(50);

/// Главная структура приложения OSIX Firmware.
pub struct App<'d> {
    board: Board<'d>,
    config: AppConfig,
}

impl<'d> App<'d> {
    /// Создаёт приложение: инициализирует логирование, конфигурацию по
    /// умолчанию и плату.
    ///
    /// Порядок важен: логирование должно быть готово раньше остальных
    /// подсистем, чтобы их собственная инициализация могла логировать
    /// диагностические сообщения.
    pub fn new() -> AppResult<Self> {
        let config = AppConfig::default();
        logger::init(logger::level_from_str("info"))?;

        log::info!(
            "запуск OSIX Firmware — станок '{}', кинематика {:?}",
            config.printer.name,
            config.printer.kinematics
        );

        let board = Board::init()?;

        Ok(Self { board, config })
    }

    /// Запускает главный цикл приложения.
    ///
    /// Возвращает ошибку только в случае неустранимой аварии платы —
    /// в обычной эксплуатации функция не завершается (`loop` работает
    /// до перезагрузки/выключения контроллера).
    pub fn run(&mut self) -> AppResult<()> {
        self.board.rgb.set_color(Color::READY)?;
        log::info!(
            "инициализация завершена: рабочая зона {:.1}x{:.1}x{:.1} мм",
            self.config.printer.bed_size.x_mm,
            self.config.printer.bed_size.y_mm,
            self.config.printer.bed_size.z_mm
        );

        loop {
            self.board.watchdog.reset()?;
            self.board.power.poll_emergency_stop()?;

            if self.board.power.is_emergency_stopped() {
                self.board.rgb.set_color(Color::ERROR)?;
            }

            std::thread::sleep(MAIN_LOOP_PERIOD);
        }
    }
}
