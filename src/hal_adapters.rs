//! Конкретные аппаратные реализации HAL-трейтов подсистем поверх
//! ESP-IDF/`esp-idf-hal`, используемые только при финальной сборке
//! ([`crate::app`]).
//!
//! Ни один из уже написанных модулей (`temperature`, `motion`, `drivers`)
//! не зависит от типов этого файла напрямую — они видят только трейты
//! ([`crate::temperature::thermistor::AnalogSample`],
//! [`crate::temperature::heater::PwmOutput`]). Это единственное место,
//! где абстракции встречаются с конкретным железом ESP32-S3.
//!
//! # Примечание о верификации
//!
//! Термистор использует классический (не "oneshot") ADC1-драйвер ESP-IDF
//! (`adc1_config_width`/`adc1_config_channel_atten`/`adc1_get_raw`) —
//! самую стабильную часть публичного API ADC, не подверженную churn'у
//! более новых Rust-обёрток `esp-idf-hal` между версиями. ШИМ, напротив,
//! использует безопасную обёртку `esp_idf_hal::ledc`, которая стабильна
//! уже много релизов подряд.

use crate::error::{AppError, AppResult};
use crate::temperature::heater::PwmOutput;
use crate::temperature::thermistor::AnalogSample;
use esp_idf_hal::ledc::LedcDriver;
use esp_idf_sys::EspError;

/// Разрядность АЦП, используемая для всех термисторов станка.
const ADC_WIDTH: esp_idf_sys::adc_bits_width_t = esp_idf_sys::adc_bits_width_t_ADC_WIDTH_BIT_12;
/// Максимальное значение 12-битного АЦП.
const ADC_MAX_VALUE: u16 = 4095;

/// Термистор, подключённый к каналу ADC1.
///
/// Оборачивает классический (не "oneshot") драйвер `adc1_*` ESP-IDF —
/// см. примечание о верификации в документации модуля.
pub struct EspAdcThermistor {
    channel: esp_idf_sys::adc1_channel_t,
}

impl EspAdcThermistor {
    /// Настраивает канал `channel` ADC1 (аттенюация 11 дБ — полный
    /// диапазон `0..=3.3` В, необходимый для делителя напряжения
    /// термистора) и возвращает считыватель.
    pub fn new(channel: u8) -> AppResult<Self> {
        // `adc1_channel_t` — тип-алиас к целому числу (генерируется
        // `bindgen` из C-перечисления), поэтому канал передаётся простым
        // приведением, а не вызовом конструктора.
        let channel: esp_idf_sys::adc1_channel_t = u32::from(channel);

        // SAFETY: обе функции глобально настраивают состояние драйвера
        // ADC1 ESP-IDF для указанного канала; не принимают указателей и не
        // имеют других предусловий, кроме однократного вызова на канал
        // (повторный вызов с теми же параметрами идемпотентен).
        unsafe {
            let ret = esp_idf_sys::adc1_config_width(ADC_WIDTH);
            EspError::convert(ret)
                .map_err(|e| AppError::Temperature(format!("не удалось настроить разрядность ADC1: {e}")))?;

            let ret = esp_idf_sys::adc1_config_channel_atten(channel, esp_idf_sys::adc_atten_t_ADC_ATTEN_DB_11);
            EspError::convert(ret)
                .map_err(|e| AppError::Temperature(format!("не удалось настроить аттенюацию канала ADC1: {e}")))?;
        }

        Ok(Self { channel })
    }
}

impl AnalogSample for EspAdcThermistor {
    fn read_raw(&mut self) -> AppResult<u16> {
        // SAFETY: канал предварительно настроен в `new()`; функция не
        // принимает указателей.
        let raw = unsafe { esp_idf_sys::adc1_get_raw(self.channel) };
        if raw < 0 {
            return Err(AppError::Temperature(format!("ошибка чтения ADC1 (код {raw})")));
        }
        Ok(raw as u16)
    }

    fn max_value(&self) -> u16 {
        ADC_MAX_VALUE
    }
}

/// ШИМ-выход поверх канала LEDC (используется нагревателями и
/// вентилятором обдува).
pub struct EspLedcPwm<'d> {
    driver: LedcDriver<'d>,
}

impl<'d> EspLedcPwm<'d> {
    /// Создаёт ШИМ-выход из уже готового канала LEDC.
    #[must_use]
    pub fn new(driver: LedcDriver<'d>) -> Self {
        Self { driver }
    }
}

impl<'d> PwmOutput for EspLedcPwm<'d> {
    fn set_duty(&mut self, duty_0_255: u8) -> AppResult<()> {
        let max_duty = self.driver.get_max_duty();
        // Масштабируем 8-битную скважность (протокол `M106`/ПИД-регуляторов)
        // на фактическую разрядность таймера LEDC, которая может быть выше.
        let scaled = (u32::from(duty_0_255) * max_duty) / u32::from(u8::MAX);
        self.driver
            .set_duty(scaled)
            .map_err(|e| AppError::board(format!("не удалось установить скважность ШИМ: {e}")))
    }
}
