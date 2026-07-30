//! Телеметрия WebSocket: формат сообщений и ограничение частоты отправки
//! (`network.toml`, раздел `websocket`, `telemetry_interval_ms`).
//!
//! Фактическая отправка кадра WebSocket зависит от конкретного API
//! HTTP-сервера, используемого для апгрейда соединения (поддержка
//! `httpd_ws_*` в `esp-idf-svc` версионно менее стабильна, чем обычные
//! HTTP-обработчики из `network::http`). Эта зависимость изолирована за
//! трейтом [`TelemetryChannel`], который финальная сборка `App` реализует
//! поверх конкретного серверного API — формат сообщений и логика
//! ограничения частоты отправки протестированы здесь независимо от него.

use crate::error::{AppError, AppResult};
use serde::Serialize;
use std::time::{Duration, Instant};

/// Положение эффектора для телеметрии.
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct PositionTelemetry {
    /// Координата X, мм.
    pub x: f32,
    /// Координата Y, мм.
    pub y: f32,
    /// Координата Z, мм.
    pub z: f32,
}

/// Состояние одного контура нагрева для телеметрии.
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct TemperatureTelemetry {
    /// Текущая измеренная температура, °C.
    pub current_c: f32,
    /// Целевая температура, °C.
    pub target_c: f32,
}

/// Снимок состояния станка, периодически транслируемый по WebSocket.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct TelemetrySnapshot {
    /// Текущее командное положение эффектора.
    pub position: PositionTelemetry,
    /// Состояние хотэнда.
    pub hotend: TemperatureTelemetry,
    /// Состояние стола.
    pub bed: TemperatureTelemetry,
    /// Прогресс текущей печати с карты памяти, `0.0..=100.0`, если печать
    /// идёт (`None`, если печати нет).
    pub print_progress_percent: Option<f32>,
    /// Время работы станка с момента включения, секунды.
    pub uptime_seconds: u64,
}

impl TelemetrySnapshot {
    /// Сериализует снимок в JSON для передачи в кадре WebSocket.
    pub fn to_json(&self) -> AppResult<String> {
        serde_json::to_string(self).map_err(|e| AppError::Network(format!("ошибка сериализации телеметрии: {e}")))
    }
}

/// Канал передачи готового JSON-сообщения подключённым клиентам WebSocket.
///
/// Реализация (конкретный вызов серверного API) подключается на этапе
/// финальной сборки `App`.
pub trait TelemetryChannel: Send {
    /// Рассылает `json` всем подключённым клиентам.
    fn broadcast(&mut self, json: &str) -> AppResult<()>;
}

/// Транслирует снимки телеметрии не чаще, чем раз в
/// `telemetry_interval_ms` (`network.toml`), независимо от того, как часто
/// вызывается [`TelemetryBroadcaster::maybe_send`].
pub struct TelemetryBroadcaster<C: TelemetryChannel> {
    channel: C,
    interval: Duration,
    last_sent: Option<Instant>,
}

impl<C: TelemetryChannel> TelemetryBroadcaster<C> {
    /// Создаёт транслятор с заданным минимальным интервалом между
    /// отправками.
    #[must_use]
    pub fn new(channel: C, interval: Duration) -> Self {
        Self { channel, interval, last_sent: None }
    }

    /// Отправляет `snapshot`, если с последней отправки прошло не меньше
    /// настроенного интервала. Возвращает `true`, если отправка
    /// действительно произошла.
    ///
    /// Рассчитан на вызов из периодической задачи с частотой выше, чем
    /// `telemetry_interval_ms` (например, из общего цикла опроса
    /// состояния) — сам решает, пора ли слать очередной кадр.
    pub fn maybe_send(&mut self, snapshot: &TelemetrySnapshot) -> AppResult<bool> {
        let now = Instant::now();
        let due = match self.last_sent {
            Some(last) => now.duration_since(last) >= self.interval,
            None => true,
        };

        if !due {
            return Ok(false);
        }

        let json = snapshot.to_json()?;
        self.channel.broadcast(&json)?;
        self.last_sent = Some(now);
        Ok(true)
    }

    /// Принудительно сбрасывает таймер, гарантируя, что следующий вызов
    /// [`TelemetryBroadcaster::maybe_send`] отправит кадр независимо от
    /// интервала (используется, например, сразу после подключения нового
    /// клиента, чтобы не заставлять его ждать первого кадра).
    pub fn force_next_send(&mut self) {
        self.last_sent = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    #[derive(Clone)]
    struct RecordingChannel {
        messages: Arc<Mutex<Vec<String>>>,
    }

    impl TelemetryChannel for RecordingChannel {
        fn broadcast(&mut self, json: &str) -> AppResult<()> {
            self.messages.lock().unwrap().push(json.to_string());
            Ok(())
        }
    }

    fn sample_snapshot() -> TelemetrySnapshot {
        TelemetrySnapshot {
            position: PositionTelemetry { x: 10.0, y: 20.0, z: 5.0 },
            hotend: TemperatureTelemetry { current_c: 200.0, target_c: 205.0 },
            bed: TemperatureTelemetry { current_c: 60.0, target_c: 60.0 },
            print_progress_percent: Some(42.5),
            uptime_seconds: 3600,
        }
    }

    #[test]
    fn first_call_always_sends() {
        let messages = Arc::new(Mutex::new(Vec::new()));
        let mut broadcaster =
            TelemetryBroadcaster::new(RecordingChannel { messages: messages.clone() }, Duration::from_secs(1));

        let sent = broadcaster.maybe_send(&sample_snapshot()).unwrap();
        assert!(sent);
        assert_eq!(messages.lock().unwrap().len(), 1);
    }

    #[test]
    fn rapid_successive_calls_are_rate_limited() {
        let messages = Arc::new(Mutex::new(Vec::new()));
        let mut broadcaster =
            TelemetryBroadcaster::new(RecordingChannel { messages: messages.clone() }, Duration::from_secs(60));

        broadcaster.maybe_send(&sample_snapshot()).unwrap();
        let sent_again = broadcaster.maybe_send(&sample_snapshot()).unwrap();

        assert!(!sent_again, "второй вызов раньше интервала не должен отправлять кадр");
        assert_eq!(messages.lock().unwrap().len(), 1);
    }

    #[test]
    fn force_next_send_bypasses_interval() {
        let messages = Arc::new(Mutex::new(Vec::new()));
        let mut broadcaster =
            TelemetryBroadcaster::new(RecordingChannel { messages: messages.clone() }, Duration::from_secs(60));

        broadcaster.maybe_send(&sample_snapshot()).unwrap();
        broadcaster.force_next_send();
        let sent_again = broadcaster.maybe_send(&sample_snapshot()).unwrap();

        assert!(sent_again);
        assert_eq!(messages.lock().unwrap().len(), 2);
    }

    #[test]
    fn snapshot_serializes_optional_progress_as_null_when_absent() {
        let mut snapshot = sample_snapshot();
        snapshot.print_progress_percent = None;
        let json = snapshot.to_json().unwrap();
        assert!(json.contains("\"print_progress_percent\":null"));
    }
}
