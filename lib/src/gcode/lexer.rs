//! Word-address lexer for DIN 66025 G-code.
//!
//! One source line = one block. Comments in parentheses `( ... )` and from
//! `;` to end of line are stripped. Words are a letter followed by a signed
//! decimal number (`G1`, `X-12.5`, `F1500.`). Whitespace between and inside
//! words is ignored (`G 01` is valid, as legacy paper-tape dialects allow).
//!
//! `E` and `F` may carry a qualifying second letter written tight against the
//! first (`FF`, `EF`, `FA`, `EU`); it is kept in [`Word::suffix`] and the
//! decoder gives it meaning.

use crate::error::GcodeError;

/// A single word, e.g. `X-12.5`.
#[derive(Debug, Clone, PartialEq)]
pub struct Word {
    /// Address letter, uppercased (`G`, `X`, `F`, ...).
    pub letter: char,
    /// Qualifying second letter of the address, uppercased, for the two-letter
    /// forms `FF`/`EF` (G0 feed and acceleration) and `F<axis>`/`E<axis>`
    /// (one auxiliary axis' feed and acceleration). `None` for plain words.
    pub suffix: Option<char>,
    /// Numeric value following the letter.
    pub value: f64,
    /// 1-based source line.
    pub line: usize,
}

impl Word {
    /// The word's address as written, e.g. `X`, `FF` or `EA`.
    pub fn name(&self) -> String {
        Self::name_of(self.letter, self.suffix)
    }

    fn name_of(letter: char, suffix: Option<char>) -> String {
        match suffix {
            Some(s) => format!("{letter}{s}"),
            None => letter.to_string(),
        }
    }
}

/// One block (source line) of words. `N` line numbers are split off.
#[derive(Debug, Clone, PartialEq)]
pub struct ParsedBlock {
    /// Program line number from an `N` word, if present.
    pub number: Option<u32>,
    /// The block's words in source order, `N` word excluded.
    pub words: Vec<Word>,
    /// 1-based source line in the input text.
    pub src_line: usize,
}

/// Lex a full program into blocks. Empty blocks (blank/comment-only lines)
/// are omitted.
pub fn lex(source: &str) -> Result<Vec<ParsedBlock>, GcodeError> {
    let mut blocks = Vec::new();
    for (idx, raw_line) in source.lines().enumerate() {
        let src_line = idx + 1;
        let (number, words) = lex_line(raw_line, src_line)?;
        if number.is_none() && words.is_empty() {
            continue;
        }
        blocks.push(ParsedBlock {
            number,
            words,
            src_line,
        });
    }
    Ok(blocks)
}

/// Byte length of the whitespace character at byte offset `i` (0 if there
/// is none).
fn whitespace_len(line: &str, i: usize) -> usize {
    match line.as_bytes().get(i) {
        Some(b' ' | b'\t') => 1,
        Some(b) if b.is_ascii() => usize::from((*b as char).is_whitespace()),
        Some(_) => line[i..]
            .chars()
            .next()
            .filter(|c| c.is_whitespace())
            .map_or(0, char::len_utf8),
        None => 0,
    }
}

/// Lex one line into its `N` number (if it leads the block) and its words.
pub(crate) fn lex_line(
    line: &str,
    src_line: usize,
) -> Result<(Option<u32>, Vec<Word>), GcodeError> {
    let bytes = line.as_bytes();
    let mut words = Vec::new();
    let mut number = None;
    // Byte offset; always on a character boundary.
    let mut i = 0;
    while let Some(&b) = bytes.get(i) {
        match b {
            // comment to end of line / program start-end marker
            b';' | b'%' => break,
            b'(' => {
                // parenthesized comment; no nesting per RS-274
                match bytes[i + 1..].iter().position(|&c| c == b')') {
                    Some(len) => i += len + 2,
                    None => return Err(GcodeError::new(src_line, "unterminated comment")),
                }
            }
            b if b.is_ascii_alphabetic() => {
                let letter = b.to_ascii_uppercase() as char;
                i += 1;
                // A second letter immediately after E or F qualifies the word
                // (`FF`/`EF` for G0, `F<axis>`/`E<axis>` for one auxiliary
                // axis). No whitespace is allowed between the two letters, so
                // an ordinary word is never mistaken for a qualified one.
                let suffix = match bytes.get(i) {
                    Some(c) if matches!(letter, 'E' | 'F') && c.is_ascii_alphabetic() => {
                        i += 1;
                        Some(c.to_ascii_uppercase() as char)
                    }
                    _ => None,
                };
                // skip whitespace between letter and number
                loop {
                    let n = whitespace_len(line, i);
                    if n == 0 {
                        break;
                    }
                    i += n;
                }
                let start = i;
                if matches!(bytes.get(i), Some(b'+' | b'-')) {
                    i += 1;
                }
                // End of the number's last character, and whether whitespace
                // is embedded in the number (`X 1 2 . 5`).
                let mut end = i;
                let mut embedded = false;
                loop {
                    if matches!(bytes.get(i), Some(b'0'..=b'9' | b'.')) {
                        embedded |= i != end;
                        i += 1;
                        end = i;
                    } else {
                        let n = whitespace_len(line, i);
                        if n == 0 {
                            break;
                        }
                        i += n;
                    }
                }
                // The common case parses straight from the source text
                let text = &line[start..end];
                let parsed = if embedded {
                    let packed: String = text.chars().filter(|c| !c.is_whitespace()).collect();
                    packed.parse::<f64>()
                } else {
                    text.parse::<f64>()
                };
                let value = match parsed {
                    // A digit string too long for an f64 parses to infinity
                    Ok(v) if v.is_finite() => v,
                    Ok(_) => {
                        let name = Word::name_of(letter, suffix);
                        return Err(GcodeError::new(
                            src_line,
                            format!("number of word '{name}' is out of range"),
                        ));
                    }
                    Err(_) => {
                        let name = Word::name_of(letter, suffix);
                        let num: String = text.chars().filter(|c| !c.is_whitespace()).collect();
                        return Err(GcodeError::new(
                            src_line,
                            format!("word '{name}' is missing a valid number (got '{num}')"),
                        ));
                    }
                };
                if letter == 'N' && number.is_none() && words.is_empty() {
                    if value < 0.0 || value.fract() != 0.0 || value > f64::from(u32::MAX) {
                        return Err(GcodeError::new(
                            src_line,
                            format!("invalid line number N{value}"),
                        ));
                    }
                    number = Some(value as u32);
                } else {
                    words.push(Word {
                        letter,
                        suffix,
                        value,
                        line: src_line,
                    });
                }
            }
            b'/' if words.is_empty() && number.is_none() => {
                // block-delete character at line start: skip the whole line
                return Ok((None, Vec::new()));
            }
            _ => {
                let n = whitespace_len(line, i);
                if n == 0 {
                    let other = line[i..].chars().next().unwrap_or('?');
                    return Err(GcodeError::new(
                        src_line,
                        format!("unexpected character '{other}'"),
                    ));
                }
                i += n;
            }
        }
    }
    Ok((number, words))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn words_and_comments() {
        let blocks = lex("N10 G1 X-12.5 Y+3. F1500 (feed) ; move\n\nG0X0Y0\n").unwrap();
        assert_eq!(blocks.len(), 2);
        assert_eq!(blocks[0].number, Some(10));
        let w = &blocks[0].words;
        assert_eq!(w.len(), 4);
        assert_eq!((w[0].letter, w[0].value), ('G', 1.0));
        assert_eq!((w[1].letter, w[1].value), ('X', -12.5));
        assert_eq!((w[2].letter, w[2].value), ('Y', 3.0));
        assert_eq!((w[3].letter, w[3].value), ('F', 1500.0));
        assert_eq!(blocks[1].src_line, 3);
        assert_eq!(blocks[1].words.len(), 3);
    }

    #[test]
    fn spaces_inside_words_and_block_delete() {
        let blocks = lex("G 0 1 X 1 2 . 5\n/ G1 X99\n").unwrap();
        assert_eq!(blocks.len(), 1);
        assert_eq!(
            (blocks[0].words[0].letter, blocks[0].words[0].value),
            ('G', 1.0)
        );
        assert_eq!(blocks[0].words[1].value, 12.5);
    }

    #[test]
    fn two_letter_words() {
        let blocks = lex("G0 FF200 EF1000 X10 FA30\n").unwrap();
        let w = &blocks[0].words;
        assert_eq!(
            (w[1].letter, w[1].suffix, w[1].value),
            ('F', Some('F'), 200.0)
        );
        assert_eq!(
            (w[2].letter, w[2].suffix, w[2].value),
            ('E', Some('F'), 1000.0)
        );
        assert_eq!((w[3].letter, w[3].suffix), ('X', None));
        assert_eq!(
            (w[4].letter, w[4].suffix, w[4].value),
            ('F', Some('A'), 30.0)
        );
        assert_eq!(w[4].name(), "FA");
        // A gap between the letters is not a qualified word: `F 50` is F50.
        let blocks = lex("G1 F 50 X1\n").unwrap();
        assert_eq!(
            (
                blocks[0].words[1].letter,
                blocks[0].words[1].suffix,
                blocks[0].words[1].value
            ),
            ('F', None, 50.0)
        );
    }

    #[test]
    fn errors() {
        assert!(lex("G1 X\n").is_err());
        assert!(lex("G1 (unterminated\n").is_err());
        assert!(lex("G1 X1 #5\n").is_err());
        assert!(lex("G1 X-\n").is_err());
        assert!(lex("G1 X1.2.3\n").is_err());
        assert!(lex("N1.5 G1 X1\n").is_err());
    }

    #[test]
    fn overflowing_number_is_rejected() {
        let e = lex(&format!("G1 X1\nG1 X{}\n", "9".repeat(400))).unwrap_err();
        assert_eq!(e.line, 2);
        assert!(e.message.contains("out of range"), "{}", e.message);
    }

    #[test]
    fn line_numbers_and_markers() {
        let blocks = lex("%\nN10\nN20 G1 X1 N5\n(only a comment)\nG1 X2 (a) Y3 ; b\n").unwrap();
        assert_eq!(blocks.len(), 3);
        assert_eq!((blocks[0].number, blocks[0].words.len()), (Some(10), 0));
        // only a leading N is the block number
        assert_eq!(blocks[1].number, Some(20));
        assert_eq!(blocks[1].words.len(), 3);
        assert_eq!(blocks[1].words[2].letter, 'N');
        assert_eq!(blocks[2].words.len(), 3);
        assert_eq!(blocks[2].src_line, 5);
    }

    #[test]
    fn non_ascii_text() {
        // inside comments anything goes; outside it is an error, not a panic
        assert_eq!(lex("G1 X1 (päätepiste ä)\n").unwrap()[0].words.len(), 2);
        let e = lex("G1 Xä\n").unwrap_err();
        assert!(
            e.message.contains("missing a valid number"),
            "{}",
            e.message
        );
        assert!(lex("G1 X1 ä\n").unwrap_err().message.contains('ä'));
        // Unicode whitespace counts as whitespace
        assert_eq!(
            lex("G1\u{a0}X1\u{2003}2\n").unwrap()[0].words[1].value,
            12.0
        );
    }
}
