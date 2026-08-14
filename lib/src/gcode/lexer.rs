//! Word-address lexer for DIN 66025 G-code.
//!
//! One source line = one block. Comments in parentheses `( ... )` and from
//! `;` to end of line are stripped. Words are a letter followed by a signed
//! decimal number (`G1`, `X-12.5`, `F1500.`). Whitespace between and inside
//! words is ignored (`G 01` is valid, as legacy paper-tape dialects allow).

use crate::error::GcodeError;

/// A single word, e.g. `X-12.5`.
#[derive(Debug, Clone, PartialEq)]
pub struct Word {
    /// Address letter, uppercased (`G`, `X`, `F`, ...).
    pub letter: char,
    /// Numeric value following the letter.
    pub value: f64,
    /// 1-based source line.
    pub line: usize,
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
        let words = lex_line(raw_line, src_line)?;
        if words.is_empty() {
            continue;
        }
        let mut number = None;
        let mut rest = Vec::with_capacity(words.len());
        for w in words {
            if w.letter == 'N' && number.is_none() && rest.is_empty() {
                if w.value < 0.0 || w.value.fract() != 0.0 {
                    return Err(GcodeError::new(
                        src_line,
                        format!("invalid line number N{}", w.value),
                    ));
                }
                number = Some(w.value as u32);
            } else {
                rest.push(w);
            }
        }
        blocks.push(ParsedBlock {
            number,
            words: rest,
            src_line,
        });
    }
    Ok(blocks)
}

fn lex_line(line: &str, src_line: usize) -> Result<Vec<Word>, GcodeError> {
    let mut words = Vec::new();
    let mut chars = line.chars().peekable();
    while let Some(&c) = chars.peek() {
        match c {
            ';' => break, // comment to end of line
            '(' => {
                // parenthesized comment; no nesting per RS-274
                chars.next();
                let mut closed = false;
                for c in chars.by_ref() {
                    if c == ')' {
                        closed = true;
                        break;
                    }
                }
                if !closed {
                    return Err(GcodeError::new(src_line, "unterminated comment"));
                }
            }
            '%' => {
                // program start/end marker - ignore the rest of the line
                break;
            }
            c if c.is_whitespace() => {
                chars.next();
            }
            c if c.is_ascii_alphabetic() => {
                let letter = c.to_ascii_uppercase();
                chars.next();
                // skip whitespace between letter and number
                while chars.peek().is_some_and(|c| c.is_whitespace()) {
                    chars.next();
                }
                let mut num = String::new();
                if chars.peek() == Some(&'+') || chars.peek() == Some(&'-') {
                    num.push(chars.next().unwrap());
                }
                while chars
                    .peek()
                    .is_some_and(|c| c.is_ascii_digit() || *c == '.' || c.is_whitespace())
                {
                    let c = chars.next().unwrap();
                    if !c.is_whitespace() {
                        num.push(c);
                    }
                }
                let value: f64 = num.parse().map_err(|_| {
                    GcodeError::new(
                        src_line,
                        format!("word '{letter}' is missing a valid number (got '{num}')"),
                    )
                })?;
                words.push(Word {
                    letter,
                    value,
                    line: src_line,
                });
            }
            '/' if words.is_empty() => {
                // block-delete character at line start: skip the whole line
                return Ok(Vec::new());
            }
            other => {
                return Err(GcodeError::new(
                    src_line,
                    format!("unexpected character '{other}'"),
                ));
            }
        }
    }
    Ok(words)
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
    fn errors() {
        assert!(lex("G1 X\n").is_err());
        assert!(lex("G1 (unterminated\n").is_err());
        assert!(lex("G1 X1 #5\n").is_err());
    }
}
