//! XML property-list decoder (the `-//Apple//DTD PLIST 1.0//EN` vocabulary).
//!
//! The tokenizer handles exactly what a plist may contain: an optional BOM,
//! the XML declaration, processing instructions, comments, a DOCTYPE without
//! an internal subset, elements with attributes, self-closing elements,
//! character data with the five predefined entities and numeric character
//! references, and CDATA sections.  An internal DTD subset (which could
//! define entities) is rejected rather than half-supported.

use super::{Dict, Format, PlistError, Value};

const MAX_DEPTH: usize = 256;

/// A start tag: name, attributes, self-closing.
type OpenTag<'a> = (&'a str, Vec<(&'a str, String)>, bool);

struct Lexer<'a> {
    s: &'a str,
    pos: usize,
}

fn line_of(s: &str, pos: usize) -> usize {
    s[..pos.min(s.len())].bytes().filter(|b| *b == b'\n').count() + 1
}

impl<'a> Lexer<'a> {
    fn err(&self, message: impl Into<String>) -> PlistError {
        PlistError { format: Format::Xml, at: line_of(self.s, self.pos), message: message.into() }
    }

    fn rest(&self) -> &'a str {
        &self.s[self.pos..]
    }

    fn eat(&mut self, lit: &str) -> bool {
        if self.rest().starts_with(lit) {
            self.pos += lit.len();
            true
        } else {
            false
        }
    }

    fn skip_ws(&mut self) {
        let r = self.rest();
        let n = r.len() - r.trim_start_matches([' ', '\t', '\r', '\n']).len();
        self.pos += n;
    }

    fn skip_until(&mut self, end: &str, what: &str) -> Result<&'a str, PlistError> {
        match self.rest().find(end) {
            Some(i) => {
                let body = &self.rest()[..i];
                self.pos += i + end.len();
                Ok(body)
            }
            None => Err(self.err(format!("unterminated {what}"))),
        }
    }

    /// Skips whitespace, comments and processing instructions.
    fn skip_misc(&mut self) -> Result<(), PlistError> {
        loop {
            self.skip_ws();
            if self.eat("<!--") {
                let body = self.skip_until("-->", "comment")?;
                if body.contains("--") {
                    return Err(self.err("'--' inside a comment"));
                }
            } else if self.rest().starts_with("<?") {
                self.pos += 2;
                self.skip_until("?>", "processing instruction")?;
            } else {
                return Ok(());
            }
        }
    }

    fn name(&mut self) -> Result<&'a str, PlistError> {
        let r = self.rest();
        let n = r
            .find(|c: char| !(c.is_ascii_alphanumeric() || c == '_' || c == '-' || c == '.' || c == ':'))
            .unwrap_or(r.len());
        if n == 0 {
            return Err(self.err("expected a name"));
        }
        self.pos += n;
        Ok(&r[..n])
    }

    /// Parses `<name attr="v" ...>` or `<name .../>` (the `<` already seen).
    fn open_tag(&mut self) -> Result<OpenTag<'a>, PlistError> {
        let name = self.name()?;
        let mut attrs = Vec::new();
        loop {
            self.skip_ws();
            if self.eat("/>") {
                return Ok((name, attrs, true));
            }
            if self.eat(">") {
                return Ok((name, attrs, false));
            }
            let key = self.name()?;
            self.skip_ws();
            if !self.eat("=") {
                return Err(self.err(format!("attribute {key} has no value")));
            }
            self.skip_ws();
            let quote = if self.eat("\"") {
                "\""
            } else if self.eat("'") {
                "'"
            } else {
                return Err(self.err(format!("attribute {key} value is not quoted")));
            };
            let raw = self.skip_until(quote, "attribute value")?;
            attrs.push((key, decode_entities(raw).map_err(|m| self.err(m))?));
        }
    }

    /// Character data up to `</name>`, with CDATA and comments allowed.
    fn text_until_close(&mut self, name: &str) -> Result<String, PlistError> {
        let mut out = String::new();
        loop {
            let r = self.rest();
            let lt = r.find('<').ok_or_else(|| self.err(format!("unterminated <{name}>")))?;
            out.push_str(&decode_entities(&r[..lt]).map_err(|m| self.err(m))?);
            self.pos += lt;
            if self.eat("<![CDATA[") {
                let body = self.skip_until("]]>", "CDATA section")?;
                out.push_str(body);
            } else if self.eat("<!--") {
                self.skip_until("-->", "comment")?;
            } else if self.eat("</") {
                let close = self.name()?;
                self.skip_ws();
                if close != name || !self.eat(">") {
                    return Err(self.err(format!("expected </{name}>, found </{close}")));
                }
                return Ok(out);
            } else {
                return Err(self.err(format!("unexpected element inside <{name}>")));
            }
        }
    }

    fn close(&mut self, name: &str) -> Result<(), PlistError> {
        self.skip_misc()?;
        if !self.eat("</") {
            return Err(self.err(format!("expected </{name}>")));
        }
        let close = self.name()?;
        self.skip_ws();
        if close != name || !self.eat(">") {
            return Err(self.err(format!("expected </{name}>, found </{close}>")));
        }
        Ok(())
    }

    /// Returns `Some((name, self_closing))` for a start tag, `None` if the next
    /// markup is an end tag (left unconsumed).
    fn next_start(&mut self) -> Result<Option<(&'a str, bool)>, PlistError> {
        self.skip_misc()?;
        if self.rest().starts_with("</") {
            return Ok(None);
        }
        if !self.eat("<") {
            return Err(self.err("expected an element"));
        }
        let (name, attrs, empty) = self.open_tag()?;
        if !attrs.is_empty() && name != "plist" {
            return Err(self.err(format!("<{name}> takes no attributes")));
        }
        Ok(Some((name, empty)))
    }

    fn value(&mut self, name: &str, empty: bool, depth: usize) -> Result<Value, PlistError> {
        if depth > MAX_DEPTH {
            return Err(self.err(format!("nesting deeper than {MAX_DEPTH}")));
        }
        let text = |lx: &mut Self| -> Result<String, PlistError> {
            if empty {
                Ok(String::new())
            } else {
                lx.text_until_close(name)
            }
        };
        Ok(match name {
            "true" | "false" => {
                if !empty {
                    let t = self.text_until_close(name)?;
                    if !t.trim().is_empty() {
                        return Err(self.err(format!("<{name}> must be empty")));
                    }
                }
                Value::Bool(name == "true")
            }
            "string" => Value::String(text(self)?),
            "integer" => {
                let t = text(self)?;
                Value::Integer(parse_integer(t.trim()).ok_or_else(|| self.err(format!("bad integer {t:?}")))?)
            }
            "real" => {
                let t = text(self)?;
                Value::Real(parse_real(t.trim()).ok_or_else(|| self.err(format!("bad real {t:?}")))?)
            }
            "data" => {
                let t = text(self)?;
                Value::Data(base64_decode(&t).map_err(|m| self.err(m))?)
            }
            "date" => {
                let t = text(self)?;
                Value::Date(parse_date(t.trim()).ok_or_else(|| self.err(format!("bad date {t:?}")))?)
            }
            "array" => {
                let mut items = Vec::new();
                if !empty {
                    while let Some((n, e)) = self.next_start()? {
                        items.push(self.value(n, e, depth + 1)?);
                    }
                    self.close("array")?;
                }
                Value::Array(items)
            }
            "dict" => {
                let mut dict = Dict::new();
                if !empty {
                    while let Some((n, e)) = self.next_start()? {
                        if n != "key" {
                            return Err(self.err(format!("expected <key> in <dict>, found <{n}>")));
                        }
                        let key = if e { String::new() } else { self.text_until_close("key")? };
                        let (vn, ve) =
                            self.next_start()?.ok_or_else(|| self.err(format!("key {key:?} has no value")))?;
                        let v = self.value(vn, ve, depth + 1)?;
                        if !dict.insert_unique(key.clone(), v) {
                            return Err(self.err(format!("duplicate dictionary key {key:?}")));
                        }
                    }
                    self.close("dict")?;
                }
                Value::Dict(dict)
            }
            other => return Err(self.err(format!("unknown plist element <{other}>"))),
        })
    }
}

pub fn parse(bytes: &[u8]) -> Result<Value, PlistError> {
    let bytes = bytes.strip_prefix(&[0xEF, 0xBB, 0xBF][..]).unwrap_or(bytes);
    let s = std::str::from_utf8(bytes).map_err(|e| PlistError {
        format: Format::Xml,
        at: line_of(&String::from_utf8_lossy(&bytes[..e.valid_up_to()]), usize::MAX),
        message: "document is not UTF-8".into(),
    })?;
    let mut lx = Lexer { s, pos: 0 };
    lx.skip_misc()?;
    if lx.eat("<!DOCTYPE") {
        let r = lx.rest();
        let end = r.find('>').ok_or_else(|| lx.err("unterminated DOCTYPE"))?;
        if r[..end].contains('[') {
            return Err(lx.err("DOCTYPE internal subsets are not supported"));
        }
        lx.pos += end + 1;
    }
    let (name, empty) = lx.next_start()?.ok_or_else(|| lx.err("no root element"))?;
    if name != "plist" {
        return Err(lx.err(format!("root element is <{name}>, not <plist>")));
    }
    if empty {
        return Err(lx.err("empty <plist>"));
    }
    let (vn, ve) = lx.next_start()?.ok_or_else(|| lx.err("<plist> holds no value"))?;
    let value = lx.value(vn, ve, 0)?;
    if lx.next_start()?.is_some() {
        return Err(lx.err("<plist> holds more than one value"));
    }
    lx.close("plist")?;
    lx.skip_misc()?;
    if !lx.rest().is_empty() {
        return Err(lx.err("content after </plist>"));
    }
    Ok(value)
}

fn decode_entities(raw: &str) -> Result<String, String> {
    if !raw.contains('&') {
        return Ok(raw.to_string());
    }
    let mut out = String::with_capacity(raw.len());
    let mut rest = raw;
    while let Some(i) = rest.find('&') {
        out.push_str(&rest[..i]);
        rest = &rest[i + 1..];
        let semi = rest.find(';').ok_or("unterminated entity reference")?;
        let ent = &rest[..semi];
        rest = &rest[semi + 1..];
        let c = match ent {
            "lt" => '<',
            "gt" => '>',
            "amp" => '&',
            "quot" => '"',
            "apos" => '\'',
            _ if ent.starts_with("#x") || ent.starts_with("#X") => u32::from_str_radix(&ent[2..], 16)
                .ok()
                .and_then(char::from_u32)
                .ok_or_else(|| format!("bad character reference &{ent};"))?,
            _ if ent.starts_with('#') => ent[1..]
                .parse::<u32>()
                .ok()
                .and_then(char::from_u32)
                .ok_or_else(|| format!("bad character reference &{ent};"))?,
            _ => return Err(format!("unknown entity &{ent};")),
        };
        out.push(c);
    }
    out.push_str(rest);
    Ok(out)
}

fn parse_integer(t: &str) -> Option<i128> {
    let (neg, body) = match t.as_bytes().first()? {
        b'-' => (true, &t[1..]),
        b'+' => (false, &t[1..]),
        _ => (false, t),
    };
    let mag = if let Some(hex) = body.strip_prefix("0x").or_else(|| body.strip_prefix("0X")) {
        // from_str_radix accepts a leading sign; a hex body must not.
        if hex.is_empty() || !hex.bytes().all(|b| b.is_ascii_hexdigit()) {
            return None;
        }
        i128::from_str_radix(hex, 16).ok()?
    } else {
        if body.is_empty() || !body.bytes().all(|b| b.is_ascii_digit()) {
            return None;
        }
        body.parse::<i128>().ok()?
    };
    if neg {
        mag.checked_neg()
    } else {
        Some(mag)
    }
}

fn parse_real(t: &str) -> Option<f64> {
    match t.to_ascii_lowercase().as_str() {
        "nan" => return Some(f64::NAN),
        "inf" | "+inf" | "infinity" | "+infinity" => return Some(f64::INFINITY),
        "-inf" | "-infinity" => return Some(f64::NEG_INFINITY),
        _ => {}
    }
    if t.is_empty() || !t.bytes().all(|b| b.is_ascii_digit() || b"+-.eE".contains(&b)) {
        return None;
    }
    t.parse::<f64>().ok()
}

/// `YYYY-MM-DDTHH:MM:SSZ` to seconds since 2001-01-01T00:00:00Z.
fn parse_date(t: &str) -> Option<f64> {
    let b = t.as_bytes();
    if b.len() != 20 || b[4] != b'-' || b[7] != b'-' || b[10] != b'T' || b[13] != b':' || b[16] != b':' || b[19] != b'Z'
    {
        return None;
    }
    let num = |r: std::ops::Range<usize>| t.get(r)?.parse::<i64>().ok();
    let (y, mo, d) = (num(0..4)?, num(5..7)?, num(8..10)?);
    let (h, mi, s) = (num(11..13)?, num(14..16)?, num(17..19)?);
    if !(1..=12).contains(&mo) || !(1..=31).contains(&d) || h > 23 || mi > 59 || s > 60 {
        return None;
    }
    // Days from civil (Howard Hinnant's algorithm), relative to 1970-01-01.
    let y2 = if mo <= 2 { y - 1 } else { y };
    let era = y2.div_euclid(400);
    let yoe = y2 - era * 400;
    let mp = (mo + 9) % 12;
    let doy = (153 * mp + 2) / 5 + d - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    let days = era * 146097 + doe - 719468;
    let unix = days * 86400 + h * 3600 + mi * 60 + s;
    Some((unix - 978_307_200) as f64)
}

pub(crate) fn base64_decode(t: &str) -> Result<Vec<u8>, String> {
    fn val(c: u8) -> Option<u8> {
        Some(match c {
            b'A'..=b'Z' => c - b'A',
            b'a'..=b'z' => c - b'a' + 26,
            b'0'..=b'9' => c - b'0' + 52,
            b'+' => 62,
            b'/' => 63,
            _ => return None,
        })
    }
    let chars: Vec<u8> = t.bytes().filter(|c| !c.is_ascii_whitespace()).collect();
    if chars.len() % 4 != 0 {
        return Err("base64 length is not a multiple of 4".into());
    }
    let mut out = Vec::with_capacity(chars.len() / 4 * 3);
    for (i, q) in chars.chunks(4).enumerate() {
        let last = i == chars.len() / 4 - 1;
        let pad = q.iter().rev().take_while(|c| **c == b'=').count();
        if pad > 2 || (pad > 0 && !last) {
            return Err("misplaced base64 padding".into());
        }
        let mut n = 0u32;
        for (j, c) in q.iter().enumerate() {
            let v = if j >= 4 - pad {
                0
            } else {
                val(*c).ok_or_else(|| format!("bad base64 character {:?}", *c as char))?
            };
            n = (n << 6) | u32::from(v);
        }
        let bytes = n.to_be_bytes();
        out.extend_from_slice(&bytes[1..4 - pad]);
    }
    Ok(out)
}

pub(crate) fn base64_encode(data: &[u8]) -> String {
    const A: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut s = String::with_capacity(data.len().div_ceil(3) * 4);
    for c in data.chunks(3) {
        let n =
            (u32::from(c[0]) << 16) | (u32::from(*c.get(1).unwrap_or(&0)) << 8) | u32::from(*c.get(2).unwrap_or(&0));
        for j in 0..4 {
            if j <= c.len() {
                s.push(A[((n >> (18 - 6 * j)) & 63) as usize] as char);
            } else {
                s.push('=');
            }
        }
    }
    s
}

/// A plain XML plist writer, used by fixtures and `speaker-tuning dump`.
pub fn write(value: &Value) -> String {
    fn esc(s: &str) -> String {
        s.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;")
    }
    fn w(v: &Value, ind: usize, out: &mut String) {
        let pad = "\t".repeat(ind);
        match v {
            Value::Bool(b) => out.push_str(&format!("{pad}<{}/>\n", if *b { "true" } else { "false" })),
            Value::Integer(i) => out.push_str(&format!("{pad}<integer>{i}</integer>\n")),
            Value::Real(r) => out.push_str(&format!("{pad}<real>{r:?}</real>\n")),
            Value::String(s) => out.push_str(&format!("{pad}<string>{}</string>\n", esc(s))),
            Value::Data(d) => out.push_str(&format!("{pad}<data>{}</data>\n", base64_encode(d))),
            Value::Date(d) => out.push_str(&format!("{pad}<real>{d:?}</real>\n")),
            Value::Uid(u) => out.push_str(&format!("{pad}<integer>{u}</integer>\n")),
            Value::Null => out.push_str(&format!("{pad}<string></string>\n")),
            Value::Array(a) => {
                out.push_str(&format!("{pad}<array>\n"));
                for i in a {
                    w(i, ind + 1, out);
                }
                out.push_str(&format!("{pad}</array>\n"));
            }
            Value::Dict(d) => {
                out.push_str(&format!("{pad}<dict>\n"));
                for (k, v) in d.iter() {
                    out.push_str(&format!("{pad}\t<key>{}</key>\n", esc(k)));
                    w(v, ind + 1, out);
                }
                out.push_str(&format!("{pad}</dict>\n"));
            }
        }
    }
    let mut out = String::from(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<!DOCTYPE plist PUBLIC \"-//Apple//DTD PLIST 1.0//EN\" \"http://www.apple.com/DTDs/PropertyList-1.0.dtd\">\n<plist version=\"1.0\">\n",
    );
    w(value, 0, &mut out);
    out.push_str("</plist>\n");
    out
}
