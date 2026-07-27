//! Парсер строк G-Code.
//!
//! Разбирает одну текстовую строку в структурированную [`GcodeCommand`]:
//! убирает комментарии (`;` и `(...)`), опционально проверяет контрольную
//! сумму (`*nn`), извлекает номер строки (`N<num>`) и параметры (`X10.5`,
//! `F1500`, ...). Не содержит знаний о конкретных командах (`G1`, `M104`, ...)
//! — их семантика находится в `gcode::commands`/`gcode::executor`.

use crate::error::{AppError, AppResult};
use std::collections::BTreeMap;

/// Разобранная команда G-Code.
#[derive(Debug, Clone, PartialEq)]
pub struct GcodeCommand {
    /// Буква команды: `'G'` или `'M'`.
    pub letter: char,
    /// Числовой код команды (например, `1` для `G1`, `104` для `M104`).
    pub code: u32,
    /// Параметры команды (буква → значение), например `{'X': 10.0, 'F': 1500.0}`.
    pub parameters: BTreeMap<char, f32>,
    /// Номер строки, если присутствовал префикс `N<num>`.
    pub line_number: Option<u32>,
}

impl GcodeCommand {
    /// Возвращает значение параметра `letter`, если он присутствует.
    #[must_use]
    pub fn get(&self, letter: char) -> Option<f32> {
        self.parameters.get(&letter.to_ascii_uppercase()).copied()
    }

    /// Возвращает `true`, если параметр `letter` присутствует в команде
    /// (значение не важно — используется для флагов вида `G28 X`).
    #[must_use]
    pub fn has(&self, letter: char) -> bool {
        self.parameters.contains_key(&letter.to_ascii_uppercase())
    }
}

/// Вычисляет контрольную сумму G-Code (XOR всех байт строки) —
/// стандартный алгоритм RepRap/Marlin для поля `*nn`.
#[must_use]
fn compute_checksum(data: &str) -> u8 {
    data.bytes().fold(0u8, |acc, byte| acc ^ byte)
}

/// Удаляет комментарии G-Code из строки: всё после `;`, а также любые
/// фрагменты в круглых скобках `(...)`.
fn strip_comments(line: &str) -> String {
    let without_semicolon = line.split(';').next().unwrap_or("");

    let mut result = String::with_capacity(without_semicolon.len());
    let mut depth = 0u32;
    for ch in without_semicolon.chars() {
        match ch {
            '(' => depth += 1,
            ')' => depth = depth.saturating_sub(1),
            _ if depth == 0 => result.push(ch),
            _ => {}
        }
    }
    result
}

/// Разбирает одну строку G-Code. Возвращает `Ok(None)` для пустых строк и
/// строк, состоящих только из комментария.
pub fn parse_line(raw_line: &str) -> AppResult<Option<GcodeCommand>> {
    let line = raw_line.trim_end_matches(['\r', '\n']);

    // Контрольная сумма (`*nn`), если присутствует, вычисляется над всей
    // частью строки до символа `*` включительно комментариев (стандарт
    // RepRap требует считать контрольную сумму до удаления комментариев,
    // если строка не содержит `;` до `*` — на практике комментарии и
    // контрольная сумма почти никогда не сочетаются в одной строке).
    let (body_with_number, expected_checksum) = match line.rsplit_once('*') {
        Some((body, checksum_str)) => {
            let expected: u8 = checksum_str
                .trim()
                .parse()
                .map_err(|_| gcode_error(0, format!("некорректная контрольная сумма: '{checksum_str}'")))?;
            (body, Some(expected))
        }
        None => (line, None),
    };

    if let Some(expected) = expected_checksum {
        let actual = compute_checksum(body_with_number);
        if actual != expected {
            return Err(gcode_error(
                0,
                format!("несовпадение контрольной суммы (ожидалось {expected}, вычислено {actual})"),
            ));
        }
    }

    let cleaned = strip_comments(body_with_number);
    let trimmed = cleaned.trim();
    if trimmed.is_empty() {
        return Ok(None);
    }

    let mut words = trimmed.split_whitespace();
    let mut line_number = None;

    let mut first_word = words.next();

    if let Some(word) = first_word {
        if word.len() > 1 && (word.starts_with('N') || word.starts_with('n')) {
            if let Ok(n) = word[1..].parse::<u32>() {
                line_number = Some(n);
                first_word = words.next();
            }
        }
    }

    let command_word = first_word.ok_or_else(|| gcode_error(line_number.unwrap_or(0), "пустая команда после номера строки".to_string()))?;

    let mut chars = command_word.chars();
    let letter = chars
        .next()
        .ok_or_else(|| gcode_error(line_number.unwrap_or(0), "команда без буквы".to_string()))?
        .to_ascii_uppercase();

    if letter != 'G' && letter != 'M' {
        return Err(gcode_error(
            line_number.unwrap_or(0),
            format!("неизвестная буква команды '{letter}' (ожидались 'G' или 'M')"),
        ));
    }

    let code: u32 = chars
        .as_str()
        .parse()
        .map_err(|_| gcode_error(line_number.unwrap_or(0), format!("некорректный код команды '{command_word}'")))?;

    let mut parameters = BTreeMap::new();
    for word in words {
        let mut param_chars = word.chars();
        let param_letter = param_chars
            .next()
            .ok_or_else(|| gcode_error(line_number.unwrap_or(0), "пустой параметр".to_string()))?
            .to_ascii_uppercase();
        let value_str = param_chars.as_str();
        let value: f32 = if value_str.is_empty() {
            // Параметры-флаги без значения (например, `G28 X`) трактуются
            // как `0.0` — значение не используется, важен сам факт наличия
            // буквы (`GcodeCommand::has`).
            0.0
        } else {
            value_str
                .parse()
                .map_err(|_| gcode_error(line_number.unwrap_or(0), format!("некорректное значение параметра '{word}'")))?
        };
        parameters.insert(param_letter, value);
    }

    Ok(Some(GcodeCommand {
        letter,
        code,
        parameters,
        line_number,
    }))
}

fn gcode_error(line: u32, reason: String) -> AppError {
    AppError::GCode { line, reason }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_simple_move_command() {
        let cmd = parse_line("G1 X10.5 Y-3 F1500").unwrap().unwrap();
        assert_eq!(cmd.letter, 'G');
        assert_eq!(cmd.code, 1);
        assert_eq!(cmd.get('X'), Some(10.5));
        assert_eq!(cmd.get('Y'), Some(-3.0));
        assert_eq!(cmd.get('F'), Some(1500.0));
    }

    #[test]
    fn parses_line_number_prefix() {
        let cmd = parse_line("N42 G28").unwrap().unwrap();
        assert_eq!(cmd.line_number, Some(42));
        assert_eq!(cmd.letter, 'G');
        assert_eq!(cmd.code, 28);
    }

    #[test]
    fn strips_semicolon_and_parenthesis_comments() {
        let cmd = parse_line("G1 X10 ; move to X10").unwrap().unwrap();
        assert_eq!(cmd.get('X'), Some(10.0));

        let cmd2 = parse_line("G1 (comment) X20 (another)").unwrap().unwrap();
        assert_eq!(cmd2.get('X'), Some(20.0));
    }

    #[test]
    fn comment_only_line_returns_none() {
        assert!(parse_line("; just a comment").unwrap().is_none());
        assert!(parse_line("   ").unwrap().is_none());
    }

    #[test]
    fn flag_style_parameter_without_value_defaults_to_zero() {
        let cmd = parse_line("G28 X Y").unwrap().unwrap();
        assert!(cmd.has('X'));
        assert!(cmd.has('Y'));
        assert!(!cmd.has('Z'));
        assert_eq!(cmd.get('X'), Some(0.0));
    }

    #[test]
    fn valid_checksum_is_accepted() {
        let body = "N3 G1 X10 Y20";
        let checksum = compute_checksum(body);
        let line = format!("{body}*{checksum}");
        let cmd = parse_line(&line).unwrap().unwrap();
        assert_eq!(cmd.line_number, Some(3));
    }

    #[test]
    fn invalid_checksum_is_rejected() {
        let line = "N3 G1 X10 Y20*1";
        assert!(parse_line(line).is_err());
    }

    #[test]
    fn unknown_command_letter_is_rejected() {
        assert!(parse_line("T0").is_err());
    }
}
