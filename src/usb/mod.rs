//! Консольный протокол поверх последовательного порта: сборка входящих
//! байт в строки G-Code и форматирование ответов (`ok`/`Error: ...`) в
//! стиле, привычном хостовым программам управления принтером
//! (OctoPrint, Pronterface, Cura) — они ожидают строку `ok` после каждой
//! успешно принятой команды.
//!
//! `dead_code` временно отключён: модуль полностью реализован и покрыт
//! тестами (на фиктивном `SerialTransport`), но ещё не создаётся `App` —
//! `UsbCdc::install()` появится в цикле задачи при финальной сборке
//! прошивки.

pub mod serial;

use crate::error::AppResult;
pub use serial::UsbCdc;

/// Максимальная длина одной строки, которую [`SerialConsole`] готов
/// накапливать в буфере. Превышение (например, из-за оборванной связи и
/// потери символа перевода строки) приводит к отбрасыванию лишних байт до
/// следующего `\n` — предохраняет от неограниченного роста буфера, а не от
/// пропуска команд в штатной работе (реальные строки G-Code кратно короче).
const MAX_LINE_LENGTH: usize = 256;

/// Источник/приёмник байт для [`SerialConsole`].
///
/// Обобщён отдельным трейтом (тот же принцип, что и у
/// [`crate::motion::step_generator::StepClock`],
/// [`crate::temperature::heater::PwmOutput`] и других HAL-абстракций
/// проекта), чтобы логику сборки строк и протокола ответов можно было
/// протестировать на хосте без реального USB CDC.
pub trait SerialTransport: Send {
    /// Считывает доступные байты, не блокируясь. Возвращает `0`, если
    /// новых данных нет.
    fn read_available(&mut self, buf: &mut [u8]) -> AppResult<usize>;
    /// Передаёт все байты `bytes`.
    fn write_all(&mut self, bytes: &[u8]) -> AppResult<()>;
}

impl SerialTransport for UsbCdc {
    fn read_available(&mut self, buf: &mut [u8]) -> AppResult<usize> {
        self.read(buf)
    }

    fn write_all(&mut self, bytes: &[u8]) -> AppResult<()> {
        self.write(bytes)
    }
}

/// Собирает входящие байты в строки G-Code и форматирует исходящие ответы.
pub struct SerialConsole<T: SerialTransport> {
    transport: T,
    line_buffer: Vec<u8>,
    read_chunk: [u8; 64],
}

impl<T: SerialTransport> SerialConsole<T> {
    /// Создаёт консоль поверх уже готового транспорта.
    #[must_use]
    pub fn new(transport: T) -> Self {
        Self {
            transport,
            line_buffer: Vec::with_capacity(MAX_LINE_LENGTH),
            read_chunk: [0u8; 64],
        }
    }

    /// Считывает доступные данные и возвращает одну завершённую строку
    /// (без символов `\r`/`\n`), если она успела накопиться. Возвращает
    /// `Ok(None)`, если новых полных строк пока нет — метод не блокирует
    /// вызывающий код и рассчитан на периодический вызов из цикла задачи.
    pub fn poll_line(&mut self) -> AppResult<Option<String>> {
        loop {
            let read = self.transport.read_available(&mut self.read_chunk)?;
            if read == 0 {
                return Ok(None);
            }

            for &byte in &self.read_chunk[..read] {
                if byte == b'\n' {
                    let line = String::from_utf8_lossy(&self.line_buffer)
                        .trim_end_matches('\r')
                        .to_string();
                    self.line_buffer.clear();
                    return Ok(Some(line));
                }
                if self.line_buffer.len() < MAX_LINE_LENGTH {
                    self.line_buffer.push(byte);
                }
                // Байты сверх `MAX_LINE_LENGTH` отбрасываются молча до
                // следующего `\n` — восстановление синхронизации без
                // разрастания буфера.
            }
        }
    }

    /// Отправляет строку `ok` — подтверждение успешной обработки команды,
    /// ожидаемое хостовыми программами управления принтером после каждой
    /// строки G-Code.
    pub fn send_ok(&mut self) -> AppResult<()> {
        self.transport.write_all(b"ok\n")
    }

    /// Отправляет произвольную строку с завершающим переводом строки
    /// (используется для содержимого отчётов `M105`/`M114`/`M115`/`M119`
    /// перед `ok`).
    pub fn send_line(&mut self, text: &str) -> AppResult<()> {
        self.transport.write_all(text.as_bytes())?;
        self.transport.write_all(b"\n")
    }

    /// Отправляет сообщение об ошибке в формате, распознаваемом хостовыми
    /// программами (`Error: <причина>`).
    pub fn send_error(&mut self, reason: &str) -> AppResult<()> {
        self.transport.write_all(b"Error: ")?;
        self.transport.write_all(reason.as_bytes())?;
        self.transport.write_all(b"\n")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;

    /// Фиктивный транспорт: очередь входящих байт + журнал исходящих.
    struct FakeTransport {
        incoming: VecDeque<u8>,
        outgoing: Vec<u8>,
    }

    impl FakeTransport {
        fn with_incoming(data: &[u8]) -> Self {
            Self {
                incoming: data.iter().copied().collect(),
                outgoing: Vec::new(),
            }
        }
    }

    impl SerialTransport for FakeTransport {
        fn read_available(&mut self, buf: &mut [u8]) -> AppResult<usize> {
            let mut count = 0;
            while count < buf.len() {
                match self.incoming.pop_front() {
                    Some(byte) => {
                        buf[count] = byte;
                        count += 1;
                    }
                    None => break,
                }
            }
            Ok(count)
        }

        fn write_all(&mut self, bytes: &[u8]) -> AppResult<()> {
            self.outgoing.extend_from_slice(bytes);
            Ok(())
        }
    }

    #[test]
    fn poll_line_returns_none_when_no_newline_yet() {
        let mut console = SerialConsole::new(FakeTransport::with_incoming(b"G28"));
        assert_eq!(console.poll_line().unwrap(), None);
    }

    #[test]
    fn poll_line_extracts_complete_line() {
        let mut console = SerialConsole::new(FakeTransport::with_incoming(b"G28\n"));
        assert_eq!(console.poll_line().unwrap(), Some("G28".to_string()));
    }

    #[test]
    fn poll_line_strips_trailing_carriage_return() {
        let mut console = SerialConsole::new(FakeTransport::with_incoming(b"G28\r\n"));
        assert_eq!(console.poll_line().unwrap(), Some("G28".to_string()));
    }

    #[test]
    fn poll_line_handles_multiple_lines_across_calls() {
        let mut console = SerialConsole::new(FakeTransport::with_incoming(b"G28\nG1 X10\n"));
        assert_eq!(console.poll_line().unwrap(), Some("G28".to_string()));
        assert_eq!(console.poll_line().unwrap(), Some("G1 X10".to_string()));
        assert_eq!(console.poll_line().unwrap(), None);
    }

    #[test]
    fn send_ok_writes_expected_bytes() {
        let mut console = SerialConsole::new(FakeTransport::with_incoming(b""));
        console.send_ok().unwrap();
        assert_eq!(console.transport.outgoing, b"ok\n");
    }

    #[test]
    fn send_error_formats_with_prefix() {
        let mut console = SerialConsole::new(FakeTransport::with_incoming(b""));
        console.send_error("неизвестная команда").unwrap();
        assert_eq!(console.transport.outgoing, "Error: неизвестная команда\n".as_bytes());
    }

    #[test]
    fn overlong_line_is_truncated_but_recovers_on_next_newline() {
        let mut long_prefix = vec![b'X'; MAX_LINE_LENGTH + 50];
        long_prefix.push(b'\n');
        long_prefix.extend_from_slice(b"G28\n");

        let mut console = SerialConsole::new(FakeTransport::with_incoming(&long_prefix));
        let first = console.poll_line().unwrap().unwrap();
        assert_eq!(first.len(), MAX_LINE_LENGTH);
        assert_eq!(console.poll_line().unwrap(), Some("G28".to_string()));
    }
}
