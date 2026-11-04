// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! The `.dspg` macro preprocessor.
//!
//! Bracket forms, evaluated in file order:
//!
//! * `[def NAME VALUE...]`: define a macro (its value is expanded at use);
//! * `[NAME]`: expand a macro (or a configuration substitution);
//! * `[if COND {THEN} {ELSE}]`, else-branch optional, branches may span
//!   lines and hold any statements;
//! * `[== A B]`, `[!= A B]`: comparisons (numeric when both sides are
//!   numbers, textual otherwise), yielding `1` or `0`.
//!
//! A condition that depends on an unresolved macro is an error: the reader
//! never picks a branch by guessing.  Any other bracket form is an error.

use crate::error::{Error, Result};
use std::collections::BTreeMap;

#[derive(Clone, Debug, PartialEq)]
pub(super) enum Tok {
    Word(String),
    Str(String),
    Open,
    Close,
    Unresolved(String),
    Newline,
}

#[derive(Clone, Debug, PartialEq)]
pub(super) enum Node {
    Tok(Tok, usize),
    Bracket(Vec<Node>, usize),
    Brace(Vec<Node>, usize),
}

fn is_word(c: char) -> bool {
    c.is_ascii_alphanumeric()
        || matches!(c, '_' | '.' | '-' | '+' | '/' | '#' | '*' | ':' | '=' | ',' | '!' | '<' | '>' | '&' | '|')
}

#[derive(Clone, Debug, PartialEq)]
enum Lex {
    T(Tok),
    LBracket,
    RBracket,
    LBrace,
    RBrace,
}

fn lex(text: &str, ctx: &str) -> Result<Vec<(Lex, usize)>> {
    let mut out = Vec::new();
    for (n, line) in text.lines().enumerate() {
        let ln = n + 1;
        let mut it = line.char_indices().peekable();
        while let Some(&(i, c)) = it.peek() {
            match c {
                ';' => break,
                c if c.is_whitespace() => {
                    it.next();
                }
                '(' | ')' | '[' | ']' | '{' | '}' => {
                    it.next();
                    out.push((
                        match c {
                            '(' => Lex::T(Tok::Open),
                            ')' => Lex::T(Tok::Close),
                            '[' => Lex::LBracket,
                            ']' => Lex::RBracket,
                            '{' => Lex::LBrace,
                            _ => Lex::RBrace,
                        },
                        ln,
                    ));
                }
                '"' => {
                    it.next();
                    let mut end = None;
                    for (j, d) in it.by_ref() {
                        if d == '"' {
                            end = Some(j);
                            break;
                        }
                    }
                    let end = end.ok_or_else(|| Error::format(ctx, format!("line {ln}: unterminated string")))?;
                    out.push((Lex::T(Tok::Str(line[i + 1..end].to_string())), ln));
                }
                c if is_word(c) => {
                    let mut end = line.len();
                    while let Some(&(j, d)) = it.peek() {
                        if !is_word(d) {
                            end = j;
                            break;
                        }
                        it.next();
                    }
                    out.push((Lex::T(Tok::Word(line[i..end].to_string())), ln));
                }
                other => return Err(Error::format(ctx, format!("line {ln}: unexpected character {other:?}"))),
            }
        }
        out.push((Lex::T(Tok::Newline), ln));
    }
    Ok(out)
}

fn tree(lx: &[(Lex, usize)], i: &mut usize, close: Option<&Lex>, ctx: &str) -> Result<Vec<Node>> {
    let mut out = Vec::new();
    while *i < lx.len() {
        let (t, ln) = &lx[*i];
        *i += 1;
        match t {
            Lex::T(tok) => out.push(Node::Tok(tok.clone(), *ln)),
            Lex::LBracket => out.push(Node::Bracket(tree(lx, i, Some(&Lex::RBracket), ctx)?, *ln)),
            Lex::LBrace => out.push(Node::Brace(tree(lx, i, Some(&Lex::RBrace), ctx)?, *ln)),
            Lex::RBracket | Lex::RBrace => {
                if close == Some(t) {
                    return Ok(out);
                }
                return Err(Error::format(
                    ctx,
                    format!("line {ln}: unbalanced {}", if *t == Lex::RBrace { '}' } else { ']' }),
                ));
            }
        }
    }
    match close {
        None => Ok(out),
        Some(c) => Err(Error::format(ctx, format!("unterminated {}", if *c == Lex::RBrace { '{' } else { '[' }))),
    }
}

pub(super) struct Pre<'a> {
    pub macros: BTreeMap<String, Vec<Node>>,
    pub subst: &'a BTreeMap<String, String>,
    pub unresolved: Vec<String>,
    pub notes: Vec<String>,
    ctx: &'a str,
}

fn significant(nodes: &[Node]) -> Vec<&Node> {
    nodes.iter().filter(|n| !matches!(n, Node::Tok(Tok::Newline, _))).collect()
}

impl<'a> Pre<'a> {
    fn expand(&mut self, nodes: &[Node], out: &mut Vec<(Tok, usize)>, depth: usize) -> Result<()> {
        if depth > 64 {
            return Err(Error::format(self.ctx, "macro expansion deeper than 64 (recursive macro?)"));
        }
        for n in nodes {
            match n {
                Node::Tok(t, ln) => out.push((t.clone(), *ln)),
                Node::Brace(_, ln) => {
                    return Err(Error::format(self.ctx, format!("line {ln}: braces outside an [if]")));
                }
                Node::Bracket(inner, ln) => self.bracket(inner, *ln, out, depth)?,
            }
        }
        Ok(())
    }

    fn bracket(&mut self, inner: &[Node], ln: usize, out: &mut Vec<(Tok, usize)>, depth: usize) -> Result<()> {
        let sig = significant(inner);
        let head = match sig.first() {
            Some(Node::Tok(Tok::Word(w), _)) => w.clone(),
            _ => return Err(Error::format(self.ctx, format!("line {ln}: malformed [...]"))),
        };
        match head.as_str() {
            "def" => {
                let Some(Node::Tok(Tok::Word(name), _)) = sig.get(1) else {
                    return Err(Error::format(self.ctx, format!("line {ln}: [def] without a name")));
                };
                let value: Vec<Node> = sig[2..].iter().map(|n| (*n).clone()).collect();
                if value.is_empty() {
                    return Err(Error::format(self.ctx, format!("line {ln}: [def {name}] has no value")));
                }
                if let Some(old) = self.macros.insert(name.clone(), value.clone()) {
                    if !same_text(&old, &value) {
                        self.notes.push(format!("line {ln}: macro {name} redefined"));
                    }
                }
                if self.subst.contains_key(name) {
                    self.notes.push(format!(
                        "line {ln}: the file defines {name}, overriding the configuration's substitution"
                    ));
                }
            }
            "if" => {
                let (cond, then, other) = match &sig[1..] {
                    [c, Node::Brace(t, _)] => (*c, t, None),
                    [c, Node::Brace(t, _), Node::Brace(e, _)] => (*c, t, Some(e)),
                    _ => {
                        return Err(Error::format(self.ctx, format!("line {ln}: malformed [if COND {{...}} {{...}}]")))
                    }
                };
                let v = self.value(cond, ln, depth)?;
                let truth = match v.as_str() {
                    "0" => false,
                    "1" => true,
                    other => match other.parse::<f64>() {
                        Ok(x) => x != 0.0,
                        Err(_) => {
                            return Err(Error::format(
                                self.ctx,
                                format!("line {ln}: [if] condition {other:?} is not a number"),
                            ))
                        }
                    },
                };
                if truth {
                    self.expand(then, out, depth + 1)?;
                } else if let Some(e) = other {
                    self.expand(e, out, depth + 1)?;
                }
            }
            "==" | "!=" => {
                let [_, a, b] = sig.as_slice() else {
                    return Err(Error::format(self.ctx, format!("line {ln}: [{head}] takes two operands")));
                };
                let (a, b) = (self.value(a, ln, depth)?, self.value(b, ln, depth)?);
                let eq = match (a.parse::<f64>(), b.parse::<f64>()) {
                    (Ok(x), Ok(y)) => x == y,
                    _ => a == b,
                };
                let r = if head == "==" { eq } else { !eq };
                out.push((Tok::Word(if r { "1" } else { "0" }.into()), ln));
            }
            name if sig.len() == 1 => {
                if let Some(v) = self.macros.get(name).cloned() {
                    let mut tmp = Vec::new();
                    self.expand(&v, &mut tmp, depth + 1)?;
                    out.extend(tmp.into_iter().map(|(t, _)| (t, ln)));
                } else if let Some(s) = self.subst.get(name) {
                    let words: Vec<&str> = s.split_whitespace().collect();
                    if words.len() != 1 || !words[0].chars().all(is_word) {
                        return Err(Error::format(self.ctx, format!("substitution {name}={s:?} is not a plain value")));
                    }
                    out.push((Tok::Word(words[0].to_string()), ln));
                } else {
                    if !self.unresolved.iter().any(|u| u == name) {
                        self.unresolved.push(name.to_string());
                    }
                    out.push((Tok::Unresolved(name.to_string()), ln));
                }
            }
            other => return Err(Error::format(self.ctx, format!("line {ln}: unknown bracket form [{other} ...]"))),
        }
        Ok(())
    }

    /// Evaluates one operand to a single word.
    fn value(&mut self, n: &Node, ln: usize, depth: usize) -> Result<String> {
        let mut tmp = Vec::new();
        self.expand(std::slice::from_ref(n), &mut tmp, depth + 1)?;
        match tmp.as_slice() {
            [(Tok::Word(w), _)] => Ok(w.clone()),
            [(Tok::Unresolved(u), _)] => Err(Error::format(
                self.ctx,
                format!("line {ln}: condition depends on unresolved macro {u}; supply the substitution"),
            )),
            _ => Err(Error::format(self.ctx, format!("line {ln}: operand is not a single value"))),
        }
    }

    pub fn macro_text(&self) -> BTreeMap<String, String> {
        fn text(nodes: &[Node]) -> String {
            nodes
                .iter()
                .map(|n| match n {
                    Node::Tok(Tok::Word(w), _) => w.clone(),
                    Node::Tok(Tok::Str(s), _) => format!("{s:?}"),
                    Node::Tok(Tok::Open, _) => "(".into(),
                    Node::Tok(Tok::Close, _) => ")".into(),
                    Node::Tok(_, _) => String::new(),
                    Node::Bracket(b, _) => format!("[{}]", text(b)),
                    Node::Brace(b, _) => format!("{{{}}}", text(b)),
                })
                .filter(|s| !s.is_empty())
                .collect::<Vec<_>>()
                .join(" ")
        }
        self.macros.iter().map(|(k, v)| (k.clone(), text(v))).collect()
    }
}

fn same_text(a: &[Node], b: &[Node]) -> bool {
    fn strip(n: &[Node]) -> Vec<Node> {
        n.iter()
            .map(|x| match x {
                Node::Tok(t, _) => Node::Tok(t.clone(), 0),
                Node::Bracket(b, _) => Node::Bracket(strip(b), 0),
                Node::Brace(b, _) => Node::Brace(strip(b), 0),
            })
            .collect()
    }
    strip(a) == strip(b)
}

/// Statements after preprocessing: (first line, tokens), blank lines dropped.
pub(super) struct Output {
    pub statements: Vec<(usize, Vec<Tok>)>,
    pub macros: BTreeMap<String, String>,
    pub unresolved: Vec<String>,
    pub notes: Vec<String>,
}

pub(super) fn run(text: &str, subst: &BTreeMap<String, String>, ctx: &str) -> Result<Output> {
    let lx = lex(text, ctx)?;
    let mut i = 0;
    let nodes = tree(&lx, &mut i, None, ctx)?;
    let mut pre = Pre { macros: BTreeMap::new(), subst, unresolved: Vec::new(), notes: Vec::new(), ctx };
    let mut toks = Vec::new();
    pre.expand(&nodes, &mut toks, 0)?;
    let mut statements = Vec::new();
    let mut cur: Vec<Tok> = Vec::new();
    let mut start = 0;
    for (t, ln) in toks {
        if t == Tok::Newline {
            if !cur.is_empty() {
                statements.push((start, std::mem::take(&mut cur)));
            }
        } else {
            if cur.is_empty() {
                start = ln;
            }
            cur.push(t);
        }
    }
    if !cur.is_empty() {
        statements.push((start, cur));
    }
    Ok(Output { statements, macros: pre.macro_text(), unresolved: pre.unresolved, notes: pre.notes })
}
