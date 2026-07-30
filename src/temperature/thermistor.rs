//! Чтение температуры термистора: перевод показаний АЦП в сопротивление
//! (делитель напряжения), затем сопротивления — в температуру по
//! уравнению Бета (упрощённая модель Стейнхарта-Харта, стандартная для
//! термисторов, используемых в 3D-печати).

use crate::config::temperature::{ThermistorConfig, ThermistorModel};
use crate::error::{AppError, AppResult};

/// Источник сырых показаний АЦП.
///
/// Обобщён отдельным трейтом (а не завязан на конкретный
/// `esp_idf_hal::adc`), чтобы формулы преобразования сопротивление →
/// температура можно было покрыть хостовыми тестами с фиктивным `AnalogSample`.
pub trait AnalogSample: Send {
    /// Считывает одно сырое значение АЦП.
    fn read_raw(&mut self) -> AppResult<u16>;
    /// Максимальное значение АЦП (например, `4095` для 12-битного АЦП
    /// ESP32-S3), соответствующее полному опорному напряжению.
    fn max_value(&self) -> u16;
}

/// Параметры уравнения Бета для конкретной модели термистора:
/// `(R0 при 25°C, Ом; коэффициент Beta)`.
fn beta_parameters(model: ThermistorModel) -> (f32, f32) {
    match model {
        ThermistorModel::Ntc100K3950 => (100_000.0, 3950.0),
        ThermistorModel::Ntc100K3435 => (100_000.0, 3435.0),
    }
}

/// Опорная температура уравнения Бета, Кельвины (`25°C`).
const REFERENCE_TEMPERATURE_K: f32 = 298.15;
/// Смещение перевода Кельвины → Цельсии.
const KELVIN_TO_CELSIUS_OFFSET: f32 = 273.15;

/// Переводит сопротивление термистора в температуру по уравнению Бета:
/// `1/T = 1/T0 + (1/B) * ln(R/R0)`.
#[must_use]
pub fn resistance_to_celsius(resistance_ohms: f32, model: ThermistorModel) -> f32 {
    let (r0, beta) = beta_parameters(model);
    let inv_t = 1.0 / REFERENCE_TEMPERATURE_K + (1.0 / beta) * (resistance_ohms / r0).ln();
    (1.0 / inv_t) - KELVIN_TO_CELSIUS_OFFSET
}

/// Термистор, подключённый через делитель напряжения (термистор — на
/// землю, резистор подтяжки `pullup_ohms` — на опорное напряжение, АЦП
/// измеряет напряжение в точке соединения).
pub struct Thermistor<A: AnalogSample> {
    adc: A,
    config: ThermistorConfig,
}

impl<A: AnalogSample> Thermistor<A> {
    /// Создаёт термистор поверх уже сконфигурированного источника отсчётов АЦП.
    #[must_use]
    pub fn new(adc: A, config: ThermistorConfig) -> Self {
        Self { adc, config }
    }

    /// Считывает температуру, усредняя `config.oversampling` отсчётов АЦП
    /// для подавления шума.
    ///
    /// Возвращает [`AppError::Temperature`], если показания АЦП находятся
    /// на границах диапазона (`0` или `max_value`) — это соответствует
    /// обрыву или короткому замыканию датчика, а не реальной температуре.
    pub fn read_celsius(&mut self) -> AppResult<f32> {
        let samples = self.config.oversampling.max(1);
        let mut sum: u32 = 0;
        for _ in 0..samples {
            sum += u32::from(self.adc.read_raw()?);
        }
        let raw = (sum / u32::from(samples)) as u16;
        let max_value = self.adc.max_value();

        if raw == 0 {
            return Err(AppError::Temperature(
                "показание АЦП термистора равно 0 — вероятен обрыв датчика".to_string(),
            ));
        }
        if raw >= max_value {
            return Err(AppError::Temperature(
                "показание АЦП термистора на максимуме — вероятно короткое замыкание".to_string(),
            ));
        }

        let ratio = f32::from(raw) / f32::from(max_value);
        // R_ntc = R_pullup / (1/ratio - 1), где `ratio` — доля опорного
        // напряжения на термисторе (делитель "термистор к земле").
        let resistance = self.config.pullup_ohms / (1.0 / ratio - 1.0);

        Ok(resistance_to_celsius(resistance, self.config.model))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct FixedAdc {
        raw: u16,
        max: u16,
    }
    impl AnalogSample for FixedAdc {
        fn read_raw(&mut self) -> AppResult<u16> {
            Ok(self.raw)
        }
        fn max_value(&self) -> u16 {
            self.max
        }
    }

    #[test]
    fn resistance_at_reference_temperature_yields_25_celsius() {
        // При R = R0 уравнение Бета должно вернуть ровно опорную температуру.
        let celsius = resistance_to_celsius(100_000.0, ThermistorModel::Ntc100K3950);
        assert!((celsius - 25.0).abs() < 1e-3);
    }

    #[test]
    fn lower_resistance_means_higher_temperature() {
        // NTC: сопротивление падает с ростом температуры.
        let hot = resistance_to_celsius(20_000.0, ThermistorModel::Ntc100K3950);
        let cold = resistance_to_celsius(200_000.0, ThermistorModel::Ntc100K3950);
        assert!(hot > 25.0);
        assert!(cold < 25.0);
        assert!(hot > cold);
    }

    #[test]
    fn open_circuit_reading_is_rejected() {
        let adc = FixedAdc { raw: 0, max: 4095 };
        let config = ThermistorConfig {
            model: ThermistorModel::Ntc100K3950,
            pullup_ohms: 4700.0,
            oversampling: 1,
        };
        let mut thermistor = Thermistor::new(adc, config);
        assert!(thermistor.read_celsius().is_err());
    }

    #[test]
    fn mid_scale_reading_produces_plausible_room_temperature() {
        // При R_pullup == R0 полушкальное показание соответствует ratio=0.5,
        // что даёт R_ntc == R_pullup == R0 == опорной температуре.
        let adc = FixedAdc { raw: 2048, max: 4095 };
        let config = ThermistorConfig {
            model: ThermistorModel::Ntc100K3950,
            pullup_ohms: 100_000.0,
            oversampling: 4,
        };
        let mut thermistor = Thermistor::new(adc, config);
        let celsius = thermistor.read_celsius().unwrap();
        assert!((celsius - 25.0).abs() < 1.0, "получено {celsius}°C");
    }
}
