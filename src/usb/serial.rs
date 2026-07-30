//! Низкоуровневая обёртка над USB CDC-ACM (TinyUSB) — виртуальный
//! последовательный порт поверх встроенного USB OTG контроллера ESP32-S3.
//!
//! # Примечание о верификации
//!
//! Стандартная C-инициализация `tinyusb_config_t` в примерах ESP-IDF часто
//! использует значения по умолчанию через `TINYUSB_CONFIG_DEFAULT()`. Как
//! и `SDSPI_HOST_DEFAULT()` в `sdcard::fatfs`, это может быть
//! функциональным макросом, не гарантированно попадающим в биндинги
//! `bindgen`. Этот модуль обнуляет структуру напрямую и полагается на то,
//! что нулевые указатели дескрипторов (`device_descriptor`,
//! `string_descriptor`, `configuration_descriptor`) означают "использовать
//! дескрипторы по умолчанию, сгенерированные `esp_tinyusb`" — так
//! документировано поведение компонента, но при первой сборке под
//! конкретную версию ESP-IDF стоит свериться с `esp_tinyusb/tinyusb.h`.

use crate::error::{AppError, AppResult};
use esp_idf_sys::EspError;
use std::mem::MaybeUninit;

/// Единственный используемый станком виртуальный последовательный порт
/// (CDC-ACM интерфейс `0`).
const CDC_ITF: esp_idf_sys::tinyusb_cdcacm_itf_t = esp_idf_sys::tinyusb_cdcacm_itf_t_TINYUSB_CDC_ACM_0;

/// Размер внутреннего кольцевого буфера приёма TinyUSB, байт.
const RX_BUFFER_SIZE: usize = 256;

/// USB CDC-ACM порт, готовый к побайтовому поллинговому чтению/записи.
///
/// Не реализует построчную сборку сама — этим занимается
/// [`crate::usb::SerialConsole`], работающий поверх трейта
/// [`crate::usb::SerialTransport`], которому эта структура и удовлетворяет.
pub struct UsbCdc;

impl UsbCdc {
    /// Устанавливает драйвер TinyUSB и инициализирует CDC-ACM интерфейс.
    ///
    /// Должна вызываться ровно один раз за время жизни программы — TinyUSB
    /// не поддерживает повторную установку драйвера без предварительного
    /// `tinyusb_driver_uninstall`.
    pub fn install() -> AppResult<Self> {
        // SAFETY: POD-структура; обнуление корректно означает "используйте
        // дескрипторы по умолчанию" для указателей `*_descriptor` и `false`
        // для `external_phy` (ESP32-S3 использует встроенный USB PHY, что
        // и является режимом по умолчанию).
        let driver_config: esp_idf_sys::tinyusb_config_t = unsafe { MaybeUninit::zeroed().assume_init() };

        // SAFETY: `driver_config` — валидная (пусть и обнулённая помимо
        // задокументированных значений по умолчанию) структура, живущая до
        // конца вызова; `tinyusb_driver_install` не сохраняет указатель на
        // неё после возврата.
        let ret = unsafe { esp_idf_sys::tinyusb_driver_install(&driver_config) };
        EspError::convert(ret)
            .map_err(|e| AppError::board(format!("не удалось установить драйвер USB TinyUSB: {e}")))?;

        let mut acm_config: esp_idf_sys::tinyusb_config_cdcacm_t = unsafe { MaybeUninit::zeroed().assume_init() };
        acm_config.usb_dev = esp_idf_sys::tinyusb_usb_device_t_TINYUSB_USBDEV_0;
        acm_config.cdc_port = CDC_ITF;
        acm_config.rx_unread_buf_sz = RX_BUFFER_SIZE;
        // Коллбэки (`callback_rx`, `callback_rx_wanted_char`,
        // `callback_line_state_changed`, `callback_line_coding_changed`)
        // намеренно оставлены нулевыми (`None`) — чтение выполняется
        // поллингом через `UsbCdc::read`, а не асинхронными коллбэками,
        // что проще интегрировать в цикл задачи `scheduler::Task`.

        // SAFETY: аналогично вызову выше — `acm_config` живёт до конца
        // функции, указатель не сохраняется вызываемой стороной.
        let ret = unsafe { esp_idf_sys::tusb_cdc_acm_init(&acm_config) };
        EspError::convert(ret)
            .map_err(|e| AppError::board(format!("не удалось инициализировать USB CDC-ACM: {e}")))?;

        log::info!("USB CDC-ACM инициализирован");
        Ok(Self)
    }

    /// Считывает доступные байты, не блокируясь. Возвращает `0`, если
    /// новых данных нет — это штатная ситуация при поллинге, а не ошибка.
    pub fn read(&mut self, buf: &mut [u8]) -> AppResult<usize> {
        let mut rx_size: usize = 0;
        // SAFETY: `buf` — валидный срез на время вызова, `rx_size`
        // указывает на локальную переменную для результата.
        let ret = unsafe { esp_idf_sys::tinyusb_cdcacm_read(CDC_ITF, buf.as_mut_ptr(), buf.len(), &mut rx_size) };

        match EspError::convert(ret) {
            Ok(()) => Ok(rx_size),
            // Отсутствие данных для чтения — не ошибка при поллинге.
            Err(e) if e.code() == esp_idf_sys::ESP_ERR_NOT_FOUND as i32 => Ok(0),
            Err(e) => Err(AppError::board(format!("ошибка чтения USB CDC: {e}"))),
        }
    }

    /// Ставит `bytes` в очередь на передачу и немедленно её отправляет
    /// (`flush` с нулевым таймаутом ожидания).
    pub fn write(&mut self, bytes: &[u8]) -> AppResult<()> {
        // SAFETY: `bytes` — валидный срез памяти на время вызова.
        let queued = unsafe { esp_idf_sys::tinyusb_cdcacm_write_queue(CDC_ITF, bytes.as_ptr(), bytes.len()) };
        if queued != bytes.len() {
            log::warn!(
                "буфер передачи USB CDC переполнен: поставлено {queued} из {} байт",
                bytes.len()
            );
        }

        // SAFETY: не принимает указателей, требующих обоснования владения.
        let ret = unsafe { esp_idf_sys::tinyusb_cdcacm_write_flush(CDC_ITF, 0) };
        EspError::convert(ret)
            .map_err(|e| AppError::board(format!("не удалось передать данные по USB CDC: {e}")))?;
        Ok(())
    }
}
