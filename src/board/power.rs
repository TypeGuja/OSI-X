//! Управление силовым питанием станка и аппаратной аварийной остановкой.

use crate::error::{AppError, AppResult};
use esp_idf_hal::gpio::{AnyIOPin, AnyOutputPin, Input, Output, PinDriver, Pull};

/// Управляет силовым реле/MOSFET-ключом блока питания (двигатели, нагреватели)
/// и читает состояние аппаратной кнопки аварийной остановки.
///
/// Инкапсулирует единственный `unsafe`-независимый доступ к GPIO питания —
/// остальной код прошивки включает/выключает питание только через методы
/// этой структуры, что исключает случайную рассинхронизацию состояния.
pub struct Power<'d> {
    psu_enable: PinDriver<'d, AnyOutputPin, Output>,
    emergency_stop: PinDriver<'d, AnyIOPin, Input>,
    enabled: bool,
}

impl<'d> Power<'d> {
    /// Инициализирует подсистему питания на основе уже сконфигурированных
    /// пинов. Питание по умолчанию выключено (безопасное состояние при
    /// старте прошивки).
    pub fn new(psu_pin: AnyOutputPin, estop_pin: AnyIOPin) -> AppResult<Self> {
        let mut psu_enable = PinDriver::output(psu_pin)
            .map_err(|e| AppError::board(format!("не удалось настроить PSU_ENABLE: {e}")))?;
        psu_enable
            .set_low()
            .map_err(|e| AppError::board(format!("не удалось выключить питание при старте: {e}")))?;

        let mut emergency_stop = PinDriver::input(estop_pin)
            .map_err(|e| AppError::board(format!("не удалось настроить EMERGENCY_STOP: {e}")))?;
        emergency_stop
            .set_pull(Pull::Up)
            .map_err(|e| AppError::board(format!("не удалось настроить подтяжку EMERGENCY_STOP: {e}")))?;

        Ok(Self {
            psu_enable,
            emergency_stop,
            enabled: false,
        })
    }

    /// Включает силовое питание станка (моторы, нагреватели).
    ///
    /// Возвращает ошибку, если аварийная кнопка в данный момент нажата —
    /// включение питания в этом состоянии небезопасно.
    pub fn enable(&mut self) -> AppResult<()> {
        if self.is_emergency_stopped() {
            return Err(AppError::board(
                "невозможно включить питание: активна аварийная остановка",
            ));
        }
        self.psu_enable
            .set_high()
            .map_err(|e| AppError::board(format!("не удалось включить питание: {e}")))?;
        self.enabled = true;
        log::info!("силовое питание включено");
        Ok(())
    }

    /// Немедленно выключает силовое питание станка.
    pub fn disable(&mut self) -> AppResult<()> {
        self.psu_enable
            .set_low()
            .map_err(|e| AppError::board(format!("не удалось выключить питание: {e}")))?;
        self.enabled = false;
        log::warn!("силовое питание выключено");
        Ok(())
    }

    /// Возвращает `true`, если питание в данный момент включено.
    #[must_use]
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Возвращает `true`, если аппаратная кнопка E-Stop нажата
    /// (сигнал активен низким уровнем).
    #[must_use]
    pub fn is_emergency_stopped(&self) -> bool {
        self.emergency_stop.is_low()
    }

    /// Проверяет состояние E-Stop и, если он активен, а питание включено —
    /// немедленно его выключает. Должна вызываться из главного цикла
    /// приложения с высокой частотой.
    pub fn poll_emergency_stop(&mut self) -> AppResult<()> {
        if self.is_emergency_stopped() && self.enabled {
            log::error!("сработала аппаратная аварийная остановка — питание отключается");
            self.disable()?;
        }
        Ok(())
    }
}
