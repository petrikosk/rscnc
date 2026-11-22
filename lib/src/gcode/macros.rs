//! Program-flow layer between the lexer and the decoder: variables,
//! expressions and control statements in the style of Fanuc Macro B.
//!
//! A line that uses none of this (no `#`, `[`, `]`, `=` and no statement
//! keyword) goes through the plain word [lexer](super::lexer) unchanged. Any
//! other line is parsed here into one [`Stmt`]:
//!
//! | Form                                 | Meaning                              |
//! |--------------------------------------|--------------------------------------|
//! | `#1 = <expr>`                        | assignment (several fit in a line)   |
//! | `X#1`, `X-#1`, `X[#1 + 2]`           | a word whose value is computed       |
//! | `GOTO n`                             | jump to the block numbered `N n`     |
//! | `IF [<expr>] GOTO n`                 | conditional jump                     |
//! | `IF [<expr>] THEN #1 = <expr>`       | conditional assignment               |
//! | `WHILE [<expr>] DO m` … `END m`      | loop (nestable)                      |
//!
//! Expressions know `+ - * /` and `MOD`, the comparisons `EQ NE GT GE LT LE`
//! (1 when true, 0 when false), the logical `AND OR XOR` (anything non-zero
//! is true) and the functions `SIN COS TAN ASIN ACOS ATAN SQRT ABS ROUND FIX
//! FUP`, angles in degrees. `ATAN[y]/[x]` is the two-argument arc tangent.
//! Brackets group (parentheses are comments). Precedence, tightest first:
//! unary sign, `* / MOD`, `+ -`, comparisons, `AND OR XOR`.
//!
//! In a line handled here a number is written without embedded whitespace.

use crate::error::GcodeError;
use crate::gcode::lexer::{lex_line, ParsedBlock, Word};
use std::collections::BTreeMap;

/// The decoder's variables: number → value. A variable that was never
/// assigned has no entry.
pub(crate) type Variables = BTreeMap<u32, f64>;

/// Binary operators of the expression language.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum BinOp {
    Add,
    Sub,
    Mul,
    Div,
    Mod,
    Eq,
    Ne,
    Gt,
    Ge,
    Lt,
    Le,
    And,
    Or,
    Xor,
}

/// One-argument functions of the expression language.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Func {
    Sin,
    Cos,
    Tan,
    Asin,
    Acos,
    Atan,
    Sqrt,
    Abs,
    Round,
    Fix,
    Fup,
}

impl Func {
    fn from_name(name: &str) -> Option<Func> {
        Some(match name {
            "SIN" => Func::Sin,
            "COS" => Func::Cos,
            "TAN" => Func::Tan,
            "ASIN" => Func::Asin,
            "ACOS" => Func::Acos,
            "ATAN" => Func::Atan,
            "SQRT" => Func::Sqrt,
            "ABS" => Func::Abs,
            "ROUND" => Func::Round,
            "FIX" => Func::Fix,
            "FUP" => Func::Fup,
            _ => return None,
        })
    }
}

/// An expression tree.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum Expr {
    /// A literal.
    Num(f64),
    /// `#<n>`: the variable whose number the inner expression gives.
    Var(Box<Expr>),
    /// Unary minus.
    Neg(Box<Expr>),
    /// A binary operation.
    Binary(BinOp, Box<Expr>, Box<Expr>),
    /// A one-argument function.
    Func(Func, Box<Expr>),
    /// `ATAN[y]/[x]`.
    Atan2(Box<Expr>, Box<Expr>),
}

/// The variable number an expression names.
pub(crate) fn variable_number(value: f64) -> Result<u32, String> {
    let rounded = value.round();
    if !value.is_finite()
        || (value - rounded).abs() > 1e-9
        || rounded < 0.0
        || rounded > f64::from(u32::MAX)
    {
        return Err(format!(
            "{value} is not a variable number (a whole number ≥ 0)"
        ));
    }
    Ok(rounded as u32)
}

impl Expr {
    /// Evaluate against the variables. Reading a variable that was never
    /// assigned, dividing by zero and results that are not finite numbers
    /// are errors.
    pub(crate) fn eval(&self, vars: &Variables) -> Result<f64, String> {
        let value = match self {
            Expr::Num(v) => *v,
            Expr::Var(index) => {
                let n = variable_number(index.eval(vars)?)?;
                *vars
                    .get(&n)
                    .ok_or_else(|| format!("variable #{n} is read before it has a value"))?
            }
            Expr::Neg(e) => -e.eval(vars)?,
            Expr::Binary(op, a, b) => {
                let (a, b) = (a.eval(vars)?, b.eval(vars)?);
                let truth = |t: bool| if t { 1.0 } else { 0.0 };
                match op {
                    BinOp::Add => a + b,
                    BinOp::Sub => a - b,
                    BinOp::Mul => a * b,
                    BinOp::Div | BinOp::Mod if b == 0.0 => {
                        return Err("division by zero".into());
                    }
                    BinOp::Div => a / b,
                    BinOp::Mod => a % b,
                    BinOp::Eq => truth(a == b),
                    BinOp::Ne => truth(a != b),
                    BinOp::Gt => truth(a > b),
                    BinOp::Ge => truth(a >= b),
                    BinOp::Lt => truth(a < b),
                    BinOp::Le => truth(a <= b),
                    BinOp::And => truth(a != 0.0 && b != 0.0),
                    BinOp::Or => truth(a != 0.0 || b != 0.0),
                    BinOp::Xor => truth((a != 0.0) != (b != 0.0)),
                }
            }
            Expr::Func(f, arg) => {
                let x = arg.eval(vars)?;
                match f {
                    Func::Sin => x.to_radians().sin(),
                    Func::Cos => x.to_radians().cos(),
                    Func::Tan => x.to_radians().tan(),
                    Func::Asin | Func::Acos if !(-1.0..=1.0).contains(&x) => {
                        return Err(format!("arc sine / arc cosine of {x} (outside [-1, 1])"));
                    }
                    Func::Asin => x.asin().to_degrees(),
                    Func::Acos => x.acos().to_degrees(),
                    Func::Atan => x.atan().to_degrees(),
                    Func::Sqrt if x < 0.0 => {
                        return Err(format!("square root of the negative number {x}"));
                    }
                    Func::Sqrt => x.sqrt(),
                    Func::Abs => x.abs(),
                    Func::Round => x.round(),
                    // FIX drops the fraction, FUP raises it to the next whole
                    // number, both in absolute value (as on a Fanuc control).
                    Func::Fix => x.trunc(),
                    Func::Fup => {
                        if x >= 0.0 {
                            x.ceil()
                        } else {
                            x.floor()
                        }
                    }
                }
            }
            Expr::Atan2(y, x) => y.eval(vars)?.atan2(x.eval(vars)?).to_degrees(),
        };
        if value.is_finite() {
            Ok(value)
        } else {
            Err("the expression has no finite value".into())
        }
    }
}

/// A word whose value is an expression (`X[#1 + 2]`).
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct WordExpr {
    pub(crate) letter: char,
    pub(crate) suffix: Option<char>,
    pub(crate) value: Expr,
}

/// One assignment `#<target> = <value>`.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Assign {
    /// Expression giving the variable's number.
    pub(crate) target: Expr,
    pub(crate) value: Expr,
}

/// What a conditional does when its condition holds.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum Then {
    Goto(Expr),
    Assign(Vec<Assign>),
}

/// One program line.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum Stmt {
    /// An ordinary block, lexed by the plain word lexer.
    Plain(ParsedBlock),
    /// A block with computed word values.
    Block(Vec<WordExpr>),
    /// `#1 = …` (one or more).
    Assign(Vec<Assign>),
    /// `GOTO n`.
    Goto(Expr),
    /// `IF [cond] GOTO n` / `IF [cond] THEN #1 = …`.
    If { cond: Expr, then: Then },
    /// `WHILE [cond] DO m`; `end` is the index of the matching `END` line
    /// (filled in by [`link_loops`]).
    While { cond: Expr, label: u32, end: usize },
    /// `END m`; `start` is the index of the matching `WHILE` line.
    End { label: u32, start: usize },
}

/// One non-empty program line.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Line {
    /// The block number from a leading `N` word.
    pub(crate) number: Option<u32>,
    /// 1-based source line.
    pub(crate) src_line: usize,
    pub(crate) stmt: Stmt,
}

/// Parse a program text into lines. Blank and comment-only lines are left
/// out, as by [`lex`](super::lexer::lex).
pub(crate) fn parse_program(source: &str) -> Result<Vec<Line>, GcodeError> {
    let mut lines = Vec::new();
    for (idx, raw) in source.lines().enumerate() {
        let src_line = idx + 1;
        if is_macro_line(raw) {
            if let Some(line) = Parser::new(raw, src_line)?.line()? {
                lines.push(line);
            }
        } else {
            let (number, words) = lex_line(raw, src_line)?;
            if number.is_none() && words.is_empty() {
                continue;
            }
            lines.push(Line {
                number,
                src_line,
                stmt: Stmt::Plain(ParsedBlock {
                    number,
                    words,
                    src_line,
                }),
            });
        }
    }
    Ok(lines)
}

/// Pair every `WHILE … DO m` with its `END m`. Returns the source line and a
/// message for a loop that is not closed, or an `END` without its `WHILE`.
pub(crate) fn link_loops(lines: &mut [Line]) -> Result<(), (usize, String)> {
    let mut open: Vec<(usize, u32)> = Vec::new();
    for i in 0..lines.len() {
        let src_line = lines[i].src_line;
        match &mut lines[i].stmt {
            Stmt::While { label, .. } => open.push((i, *label)),
            Stmt::End { label, start } => {
                let label = *label;
                match open.pop() {
                    Some((at, l)) if l == label => {
                        *start = at;
                        if let Stmt::While { end, .. } = &mut lines[at].stmt {
                            *end = i;
                        }
                    }
                    Some((at, l)) => {
                        return Err((
                            src_line,
                            format!(
                                "END {label} closes the loop DO {l} opened on line {} \
                                 (loops must nest)",
                                lines[at].src_line
                            ),
                        ));
                    }
                    None => {
                        return Err((src_line, format!("END {label} has no WHILE … DO {label}")));
                    }
                }
            }
            _ => {}
        }
    }
    match open.first() {
        Some(&(at, label)) => Err((
            lines[at].src_line,
            format!("WHILE … DO {label} has no END {label}"),
        )),
        None => Ok(()),
    }
}

/// Statement keywords. None of them is a valid start of a plain word.
const KEYWORDS: [&str; 6] = ["GOTO", "IF", "WHILE", "END", "DO", "THEN"];

/// Whether a line needs this module's parser rather than the plain lexer:
/// it has a variable, a bracket or an assignment, or a statement keyword.
/// None of these can occur in a line the plain lexer accepts.
fn is_macro_line(line: &str) -> bool {
    let bytes = line.as_bytes();
    let mut i = 0;
    let mut first = true;
    while let Some(&b) = bytes.get(i) {
        match b {
            b';' | b'%' => return false,
            // block delete: the plain lexer skips the line
            b'/' if first => return false,
            b'(' => match bytes[i + 1..].iter().position(|&c| c == b')') {
                Some(len) => i += len + 2,
                // (reported by the plain lexer)
                None => return false,
            },
            b'#' | b'[' | b']' | b'=' => return true,
            b if b.is_ascii_alphabetic() => {
                let start = i;
                while bytes.get(i).is_some_and(u8::is_ascii_alphabetic) {
                    i += 1;
                }
                let run = &line[start..i];
                if KEYWORDS.iter().any(|k| k.eq_ignore_ascii_case(run)) {
                    return true;
                }
                first = false;
            }
            _ => {
                first &= b.is_ascii_whitespace();
                i += 1;
            }
        }
    }
    false
}

#[derive(Debug, Clone, PartialEq)]
enum Tok {
    Num(f64),
    Ident(String),
    Hash,
    LBracket,
    RBracket,
    Assign,
    Plus,
    Minus,
    Star,
    Slash,
}

impl Tok {
    fn describe(&self) -> String {
        match self {
            Tok::Num(v) => format!("number {v}"),
            Tok::Ident(s) => format!("'{s}'"),
            Tok::Hash => "'#'".into(),
            Tok::LBracket => "'['".into(),
            Tok::RBracket => "']'".into(),
            Tok::Assign => "'='".into(),
            Tok::Plus => "'+'".into(),
            Tok::Minus => "'-'".into(),
            Tok::Star => "'*'".into(),
            Tok::Slash => "'/'".into(),
        }
    }
}

fn tokenize(line: &str, src_line: usize) -> Result<Vec<Tok>, GcodeError> {
    let bytes = line.as_bytes();
    let mut toks = Vec::new();
    let mut i = 0;
    while let Some(&b) = bytes.get(i) {
        match b {
            b';' | b'%' => break,
            b'(' => match bytes[i + 1..].iter().position(|&c| c == b')') {
                Some(len) => i += len + 2,
                None => return Err(GcodeError::new(src_line, "unterminated comment")),
            },
            b'#' | b'[' | b']' | b'=' | b'+' | b'-' | b'*' | b'/' => {
                toks.push(match b {
                    b'#' => Tok::Hash,
                    b'[' => Tok::LBracket,
                    b']' => Tok::RBracket,
                    b'=' => Tok::Assign,
                    b'+' => Tok::Plus,
                    b'-' => Tok::Minus,
                    b'*' => Tok::Star,
                    _ => Tok::Slash,
                });
                i += 1;
            }
            b'0'..=b'9' | b'.' => {
                let start = i;
                while matches!(bytes.get(i), Some(b'0'..=b'9' | b'.')) {
                    i += 1;
                }
                let text = &line[start..i];
                match text.parse::<f64>() {
                    Ok(v) if v.is_finite() => toks.push(Tok::Num(v)),
                    Ok(_) => {
                        return Err(GcodeError::new(
                            src_line,
                            format!("number '{text}' is out of range"),
                        ));
                    }
                    Err(_) => {
                        return Err(GcodeError::new(
                            src_line,
                            format!("'{text}' is not a valid number"),
                        ));
                    }
                }
            }
            b if b.is_ascii_alphabetic() => {
                let start = i;
                while bytes.get(i).is_some_and(u8::is_ascii_alphabetic) {
                    i += 1;
                }
                toks.push(Tok::Ident(line[start..i].to_ascii_uppercase()));
            }
            _ => {
                let c = line[i..].chars().next().unwrap_or('?');
                if !c.is_whitespace() {
                    return Err(GcodeError::new(
                        src_line,
                        format!("unexpected character '{c}'"),
                    ));
                }
                i += c.len_utf8();
            }
        }
    }
    Ok(toks)
}

struct Parser {
    toks: Vec<Tok>,
    pos: usize,
    src_line: usize,
}

impl Parser {
    fn new(line: &str, src_line: usize) -> Result<Parser, GcodeError> {
        Ok(Parser {
            toks: tokenize(line, src_line)?,
            pos: 0,
            src_line,
        })
    }

    fn err<T>(&self, message: impl Into<String>) -> Result<T, GcodeError> {
        Err(GcodeError::new(self.src_line, message))
    }

    fn peek(&self) -> Option<&Tok> {
        self.toks.get(self.pos)
    }

    fn bump(&mut self) -> Option<Tok> {
        let tok = self.toks.get(self.pos).cloned();
        self.pos += tok.is_some() as usize;
        tok
    }

    /// Consume the next token if it is `tok`.
    fn eat(&mut self, tok: &Tok) -> bool {
        let hit = self.peek() == Some(tok);
        self.pos += hit as usize;
        hit
    }

    /// Consume the next token if it is the identifier `name`.
    fn eat_ident(&mut self, name: &str) -> bool {
        let hit = matches!(self.peek(), Some(Tok::Ident(s)) if s == name);
        self.pos += hit as usize;
        hit
    }

    /// What comes next, for error messages.
    fn found(&self) -> String {
        self.peek()
            .map_or_else(|| "the end of the line".into(), Tok::describe)
    }

    fn expect(&mut self, tok: Tok, context: &str) -> Result<(), GcodeError> {
        if self.eat(&tok) {
            Ok(())
        } else {
            self.err(format!(
                "expected {} {context}, found {}",
                tok.describe(),
                self.found()
            ))
        }
    }

    fn end_of_line(&self, after: &str) -> Result<(), GcodeError> {
        match self.peek() {
            None => Ok(()),
            Some(tok) => self.err(format!("unexpected {} after {after}", tok.describe())),
        }
    }

    /// A whole number literal (loop labels).
    fn label(&mut self, of: &str) -> Result<u32, GcodeError> {
        match self.peek() {
            Some(&Tok::Num(v)) if v.fract() == 0.0 && v <= f64::from(u32::MAX) => {
                self.pos += 1;
                Ok(v as u32)
            }
            _ => self.err(format!("{of} needs a whole number, found {}", self.found())),
        }
    }

    fn line(mut self) -> Result<Option<Line>, GcodeError> {
        let src_line = self.src_line;
        let mut number = None;
        if let (Some(Tok::Ident(n)), Some(Tok::Num(v))) = (self.toks.first(), self.toks.get(1)) {
            if n == "N" {
                if v.fract() != 0.0 || *v > f64::from(u32::MAX) {
                    return self.err(format!("invalid line number N{v}"));
                }
                number = Some(*v as u32);
                self.pos = 2;
            }
        }
        let stmt = match self.peek().cloned() {
            None if number.is_none() => return Ok(None),
            None => Stmt::Plain(ParsedBlock {
                number,
                words: Vec::new(),
                src_line,
            }),
            Some(Tok::Hash) => {
                let list = self.assignments()?;
                self.end_of_line("the assignment")?;
                Stmt::Assign(list)
            }
            Some(Tok::Ident(word)) => match word.as_str() {
                "GOTO" => {
                    self.pos += 1;
                    let target = self.value("GOTO")?;
                    self.end_of_line("the GOTO target")?;
                    Stmt::Goto(target)
                }
                "IF" => {
                    self.pos += 1;
                    let cond = self.bracket("after IF")?;
                    let then = if self.eat_ident("GOTO") {
                        let target = self.value("GOTO")?;
                        self.end_of_line("the GOTO target")?;
                        Then::Goto(target)
                    } else if self.eat_ident("THEN") {
                        if self.peek() != Some(&Tok::Hash) {
                            return self.err(format!(
                                "THEN takes an assignment (#n = …), found {}",
                                self.found()
                            ));
                        }
                        let list = self.assignments()?;
                        self.end_of_line("the assignment")?;
                        Then::Assign(list)
                    } else {
                        return self.err(format!(
                            "expected GOTO or THEN after the IF condition, found {}",
                            self.found()
                        ));
                    };
                    Stmt::If { cond, then }
                }
                "WHILE" => {
                    self.pos += 1;
                    let cond = self.bracket("after WHILE")?;
                    if !self.eat_ident("DO") {
                        return self.err(format!(
                            "expected DO after the WHILE condition, found {}",
                            self.found()
                        ));
                    }
                    let label = self.label("DO")?;
                    self.end_of_line("the loop label")?;
                    Stmt::While {
                        cond,
                        label,
                        end: usize::MAX,
                    }
                }
                "END" => {
                    self.pos += 1;
                    let label = self.label("END")?;
                    self.end_of_line("the loop label")?;
                    Stmt::End {
                        label,
                        start: usize::MAX,
                    }
                }
                "DO" => return self.err("DO without WHILE"),
                "THEN" => return self.err("THEN without IF"),
                _ => Stmt::Block(self.words()?),
            },
            Some(tok) => {
                return self.err(format!(
                    "unexpected {} at the start of the block",
                    tok.describe()
                ))
            }
        };
        Ok(Some(Line {
            number,
            src_line,
            stmt,
        }))
    }

    /// The words of an NC block, up to the end of the line.
    fn words(&mut self) -> Result<Vec<WordExpr>, GcodeError> {
        let mut words = Vec::new();
        while let Some(tok) = self.bump() {
            let name = match tok {
                Tok::Ident(name) => name,
                Tok::Hash => {
                    return self.err(
                        "unexpected '#': a variable is either a word's value (X#1) or \
                         assigned in a block of its own (#1 = …)",
                    )
                }
                other => {
                    return self.err(format!(
                        "unexpected {} where a word was expected",
                        other.describe()
                    ))
                }
            };
            let mut letters = name.chars();
            let (letter, suffix) = match (letters.next(), letters.next(), letters.next()) {
                (Some(l), None, _) => (l, None),
                (Some(l @ ('E' | 'F')), Some(s), None) => (l, Some(s)),
                _ => return self.err(format!("unknown word or statement '{name}'")),
            };
            if matches!(self.peek(), None | Some(Tok::Ident(_))) {
                return self.err(format!("word '{name}' is missing its value"));
            }
            let value = self.value(&name)?;
            words.push(WordExpr {
                letter,
                suffix,
                value,
            });
        }
        Ok(words)
    }

    /// One or more `#n = <expr>`.
    fn assignments(&mut self) -> Result<Vec<Assign>, GcodeError> {
        let mut list = Vec::new();
        while self.peek() == Some(&Tok::Hash) {
            let Expr::Var(target) = self.variable()? else {
                unreachable!("variable() returns Expr::Var")
            };
            self.expect(Tok::Assign, "after the variable to assign")?;
            let value = self.expr()?;
            list.push(Assign {
                target: *target,
                value,
            });
        }
        Ok(list)
    }

    /// A word's value: a signed number, variable or bracketed expression.
    fn value(&mut self, of: &str) -> Result<Expr, GcodeError> {
        if self.eat(&Tok::Minus) {
            return Ok(Expr::Neg(Box::new(self.value(of)?)));
        }
        if self.eat(&Tok::Plus) {
            return self.value(of);
        }
        match self.peek() {
            Some(Tok::Num(v)) => {
                let v = *v;
                self.pos += 1;
                Ok(Expr::Num(v))
            }
            Some(Tok::Hash) => self.variable(),
            Some(Tok::LBracket) => self.bracket("as a value"),
            _ => self.err(format!(
                "'{of}' needs a number, a variable or a bracketed expression, found {}",
                self.found()
            )),
        }
    }

    /// `#n`, `#[expr]` or `##n`.
    fn variable(&mut self) -> Result<Expr, GcodeError> {
        self.expect(Tok::Hash, "before a variable number")?;
        let index = match self.peek() {
            Some(Tok::Num(v)) => {
                let v = *v;
                if v.fract() != 0.0 || v > f64::from(u32::MAX) {
                    return self.err(format!("invalid variable number #{v}"));
                }
                self.pos += 1;
                Expr::Num(v)
            }
            Some(Tok::LBracket) => self.bracket("as a variable number")?,
            Some(Tok::Hash) => self.variable()?,
            _ => {
                return self.err(format!(
                    "'#' must be followed by a variable number, found {}",
                    self.found()
                ))
            }
        };
        Ok(Expr::Var(Box::new(index)))
    }

    /// `[ expr ]`.
    fn bracket(&mut self, context: &str) -> Result<Expr, GcodeError> {
        self.expect(Tok::LBracket, context)?;
        let e = self.expr()?;
        self.expect(Tok::RBracket, "to close the expression")?;
        Ok(e)
    }

    /// A chain of binary operators of one precedence level.
    fn chain(
        &mut self,
        operand: fn(&mut Parser) -> Result<Expr, GcodeError>,
        operator: fn(&Tok) -> Option<BinOp>,
    ) -> Result<Expr, GcodeError> {
        let mut left = operand(self)?;
        while let Some(op) = self.peek().and_then(operator) {
            self.pos += 1;
            let right = operand(self)?;
            left = Expr::Binary(op, Box::new(left), Box::new(right));
        }
        Ok(left)
    }

    fn expr(&mut self) -> Result<Expr, GcodeError> {
        self.chain(Parser::comparison, |tok| match tok {
            Tok::Ident(s) => match s.as_str() {
                "AND" => Some(BinOp::And),
                "OR" => Some(BinOp::Or),
                "XOR" => Some(BinOp::Xor),
                _ => None,
            },
            _ => None,
        })
    }

    fn comparison(&mut self) -> Result<Expr, GcodeError> {
        self.chain(Parser::sum, |tok| match tok {
            Tok::Ident(s) => match s.as_str() {
                "EQ" => Some(BinOp::Eq),
                "NE" => Some(BinOp::Ne),
                "GT" => Some(BinOp::Gt),
                "GE" => Some(BinOp::Ge),
                "LT" => Some(BinOp::Lt),
                "LE" => Some(BinOp::Le),
                _ => None,
            },
            _ => None,
        })
    }

    fn sum(&mut self) -> Result<Expr, GcodeError> {
        self.chain(Parser::product, |tok| match tok {
            Tok::Plus => Some(BinOp::Add),
            Tok::Minus => Some(BinOp::Sub),
            _ => None,
        })
    }

    fn product(&mut self) -> Result<Expr, GcodeError> {
        self.chain(Parser::unary, |tok| match tok {
            Tok::Star => Some(BinOp::Mul),
            Tok::Slash => Some(BinOp::Div),
            Tok::Ident(s) if s == "MOD" => Some(BinOp::Mod),
            _ => None,
        })
    }

    fn unary(&mut self) -> Result<Expr, GcodeError> {
        if self.eat(&Tok::Minus) {
            return Ok(Expr::Neg(Box::new(self.unary()?)));
        }
        if self.eat(&Tok::Plus) {
            return self.unary();
        }
        match self.peek().cloned() {
            Some(Tok::Num(v)) => {
                self.pos += 1;
                Ok(Expr::Num(v))
            }
            Some(Tok::Hash) => self.variable(),
            Some(Tok::LBracket) => self.bracket("to open an expression"),
            Some(Tok::Ident(name)) => {
                let Some(func) = Func::from_name(&name) else {
                    return self.err(format!("unknown function or operand '{name}'"));
                };
                self.pos += 1;
                let arg = self.bracket(&format!("after {name}"))?;
                // ATAN[y]/[x]: the two-argument form
                if func == Func::Atan
                    && self.peek() == Some(&Tok::Slash)
                    && self.toks.get(self.pos + 1) == Some(&Tok::LBracket)
                {
                    self.pos += 1;
                    let x = self.bracket("after ATAN[…]/")?;
                    return Ok(Expr::Atan2(Box::new(arg), Box::new(x)));
                }
                Ok(Expr::Func(func, Box::new(arg)))
            }
            _ => self.err(format!("expected a value, found {}", self.found())),
        }
    }
}

/// Evaluate the words of a computed block into a plain [`ParsedBlock`].
pub(crate) fn evaluate_block(
    words: &[WordExpr],
    number: Option<u32>,
    src_line: usize,
    vars: &Variables,
) -> Result<ParsedBlock, String> {
    let mut out = Vec::with_capacity(words.len());
    for w in words {
        let value = w.value.eval(vars).map_err(|e| {
            let name: String = core::iter::once(w.letter).chain(w.suffix).collect();
            format!("value of word '{name}': {e}")
        })?;
        out.push(Word {
            letter: w.letter,
            suffix: w.suffix,
            value,
            line: src_line,
        });
    }
    Ok(ParsedBlock {
        number,
        words: out,
        src_line,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn eval(src: &str, vars: &Variables) -> Result<f64, String> {
        // parse `#9999 = <src>` and evaluate the right-hand side
        let line = Parser::new(&format!("#9999 = {src}"), 1)
            .and_then(Parser::line)
            .map_err(|e| e.message)?
            .unwrap();
        match line.stmt {
            Stmt::Assign(list) => list[0].value.eval(vars),
            other => panic!("not an assignment: {other:?}"),
        }
    }

    fn value(src: &str) -> f64 {
        eval(src, &Variables::new()).unwrap()
    }

    #[test]
    fn arithmetic_and_precedence() {
        assert_eq!(value("1 + 2 * 3"), 7.0);
        assert_eq!(value("[1 + 2] * 3"), 9.0);
        assert_eq!(value("10 - 4 - 3"), 3.0);
        assert_eq!(value("2 * -3"), -6.0);
        assert_eq!(value("-[2 + 3]"), -5.0);
        assert_eq!(value("7 MOD 4"), 3.0);
        assert_eq!(value("8 / 2 / 2"), 2.0);
        assert_eq!(value(".5 + 1."), 1.5);
        // comparisons bind tighter than the logical operators
        assert_eq!(value("1 LT 2 AND 3 GT 2"), 1.0);
        assert_eq!(value("1 GT 2 OR 3 GT 2"), 1.0);
        assert_eq!(value("1 LT 2 XOR 3 GT 2"), 0.0);
        assert_eq!(value("1 + 1 EQ 2"), 1.0);
        assert_eq!(value("3 NE 3"), 0.0);
        assert_eq!(value("3 GE 3"), 1.0);
        assert_eq!(value("3 LE 2"), 0.0);
    }

    #[test]
    fn functions_work_in_degrees() {
        assert!((value("SIN[30]") - 0.5).abs() < 1e-12);
        assert!((value("COS[60]") - 0.5).abs() < 1e-12);
        assert!((value("TAN[45]") - 1.0).abs() < 1e-12);
        assert!((value("ATAN[1]") - 45.0).abs() < 1e-12);
        assert!((value("ATAN[1]/[-1]") - 135.0).abs() < 1e-12);
        // … while a plain division after ATAN stays a division
        assert!((value("ATAN[1]/5") - 9.0).abs() < 1e-12);
        assert!((value("ASIN[0.5]") - 30.0).abs() < 1e-12);
        assert!((value("ACOS[0.5]") - 60.0).abs() < 1e-12);
        assert_eq!(value("SQRT[16]"), 4.0);
        assert_eq!(value("ABS[-2.5]"), 2.5);
        assert_eq!(value("ROUND[2.5]"), 3.0);
        assert_eq!((value("FIX[2.7]"), value("FIX[-2.7]")), (2.0, -2.0));
        assert_eq!((value("FUP[2.1]"), value("FUP[-2.1]")), (3.0, -3.0));
        assert_eq!(value("sqrt[abs[-9]]"), 3.0);
    }

    #[test]
    fn variables_and_errors() {
        let mut vars = Variables::new();
        vars.insert(1, 4.0);
        vars.insert(4, 10.0);
        assert_eq!(eval("#1 * 2", &vars).unwrap(), 8.0);
        // indirect: #[#1] and ##1 are variable 4
        assert_eq!(eval("#[#1]", &vars).unwrap(), 10.0);
        assert_eq!(eval("##1", &vars).unwrap(), 10.0);
        assert!(eval("#2", &vars).unwrap_err().contains("#2"));
        assert!(eval("1 / 0", &vars).unwrap_err().contains("division"));
        assert!(eval("1 MOD 0", &vars).is_err());
        assert!(eval("SQRT[-1]", &vars).is_err());
        assert!(eval("ASIN[2]", &vars).is_err());
        assert!(eval("#[0 - 1]", &vars).is_err());
        // syntax
        assert!(eval("1 +", &vars).is_err());
        assert!(eval("[1 + 2", &vars).is_err());
        assert!(eval("FOO[1]", &vars).is_err());
        assert!(eval("SIN 30", &vars).is_err());
        assert!(eval("1.2.3", &vars).is_err());
        assert!(eval("#1.5", &vars).is_err());
    }

    fn stmt(src: &str) -> Stmt {
        let lines = parse_program(src).unwrap();
        assert_eq!(lines.len(), 1, "{src}");
        lines.into_iter().next().unwrap().stmt
    }

    #[test]
    fn plain_lines_stay_with_the_plain_lexer() {
        for src in [
            "G1 X10 F5",
            "N10 G0 FF200 EF1000 X10 FA30",
            "G 0 1 X 1 2 . 5",
            "G1 X1 (a comment with #1 = [2])",
            "N5",
        ] {
            assert!(matches!(stmt(src), Stmt::Plain(_)), "{src}");
        }
        assert!(parse_program("/ #1 = 5\n; #2 = 3\n\n").unwrap().is_empty());
    }

    #[test]
    fn statements_parse() {
        assert!(matches!(stmt("#1 = 5"), Stmt::Assign(l) if l.len() == 1));
        assert!(matches!(stmt("#1 = 5 #2 = #1 + 1"), Stmt::Assign(l) if l.len() == 2));
        assert!(matches!(stmt("GOTO 10"), Stmt::Goto(Expr::Num(v)) if v == 10.0));
        assert!(matches!(stmt("goto#1"), Stmt::Goto(Expr::Var(_))));
        assert!(matches!(
            stmt("IF [#1 GT 2] GOTO 10"),
            Stmt::If {
                then: Then::Goto(_),
                ..
            }
        ));
        assert!(matches!(
            stmt("IF[#1EQ0]THEN#2=1"),
            Stmt::If {
                then: Then::Assign(_),
                ..
            }
        ));
        assert!(matches!(
            stmt("WHILE [#1 LT 3] DO 1"),
            Stmt::While { label: 1, .. }
        ));
        assert!(matches!(stmt("END1"), Stmt::End { label: 1, .. }));
        match stmt("N20 G1 X#1 Y-#2 Z[#1 + 1] FA[2*3] F100") {
            Stmt::Block(words) => {
                let names: Vec<_> = words.iter().map(|w| (w.letter, w.suffix)).collect();
                assert_eq!(
                    names,
                    [
                        ('G', None),
                        ('X', None),
                        ('Y', None),
                        ('Z', None),
                        ('F', Some('A')),
                        ('F', None)
                    ]
                );
            }
            other => panic!("{other:?}"),
        }
        let lines = parse_program("N20 #1 = 1\nN30 (label only) #2 = 2\n").unwrap();
        assert_eq!((lines[0].number, lines[1].number), (Some(20), Some(30)));
    }

    #[test]
    fn syntax_errors_carry_their_line() {
        for src in [
            "G1 X#",
            "X1 #5",
            "#1 5",
            "#1 = ",
            "G1 X[1 + 2",
            "G1 X]",
            "GOTO",
            "GOTO 10 20",
            "IF [1] X5",
            "IF 1 GOTO 5",
            "IF [1] THEN G1 X5",
            "WHILE [1] 1",
            "WHILE [1] DO",
            "WHILE [1] DO 1.5",
            "END",
            "DO 1",
            "THEN #1 = 2",
            "G1 XY[1]",
            "GX[1]",
            "= 5",
            "G1 X[1] 5",
            "N1.5 #1 = 2",
        ] {
            let e = parse_program(&format!("G1 F5 X1\n{src}\n")).unwrap_err();
            assert_eq!(e.line, 2, "{src}: {e}");
        }
    }

    #[test]
    fn loops_are_paired() {
        let mut lines =
            parse_program("WHILE [1] DO 1\nWHILE [1] DO 2\nEND 2\nG1 X1\nEND 1\n").unwrap();
        link_loops(&mut lines).unwrap();
        assert!(matches!(lines[0].stmt, Stmt::While { end: 4, .. }));
        assert!(matches!(lines[1].stmt, Stmt::While { end: 2, .. }));
        assert!(matches!(lines[2].stmt, Stmt::End { start: 1, .. }));
        assert!(matches!(lines[4].stmt, Stmt::End { start: 0, .. }));

        let broken = |src: &str| link_loops(&mut parse_program(src).unwrap()).unwrap_err();
        assert_eq!(broken("G1 X1\nWHILE [1] DO 1\n").0, 2);
        assert_eq!(broken("END 1\n").0, 1);
        assert_eq!(
            broken("WHILE [1] DO 1\nWHILE [1] DO 2\nEND 1\nEND 2\n").0,
            3
        );
    }
}
