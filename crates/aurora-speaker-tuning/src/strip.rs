// Based on aurora-audio-dsp by saku (MIT); see LICENSE-aurora-audio-dsp.
// Copyright (c) 2026 saku
// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Strips: `.austrip` (the AudioUnit presets of a graph's boxes) and
//! `.propstrip` (AudioUnit properties set on named boxes).
//!
//! `.austrip`: `{strips: [{effects: [{displayname, bypass, unit: {type,
//! subtype, manufacturer}, aupreset: <ClassInfo>}]}]}`.  An effect's
//! `displayname` is the name of the graph box it configures.
//!
//! `.propstrip`: `{Boxes: [{Name, Properties: [{Number, <typed value>}]}]}`,
//! where the typed value key names the property's representation (`UInt32`,
//! `Path`, ...).

use crate::aupreset::{unit_id, AuPreset, UnitId};
use crate::error::{Error, Result};
use crate::plist::{self, Dict, Value};
use serde::Serialize;

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Effect {
    /// The graph box this effect configures.
    pub box_name: String,
    pub bypass: bool,
    pub unit: UnitId,
    pub preset: AuPreset,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct AuStrip {
    /// `strips[i].effects`, in stored order.
    pub strips: Vec<Vec<Effect>>,
}

impl AuStrip {
    pub fn parse(bytes: &[u8], context: &str) -> Result<Self> {
        let (root, _) = plist::parse(bytes).map_err(|e| Error::plist(context, e))?;
        let root = root.as_dict().ok_or_else(|| Error::format(context, "root is not a dictionary"))?;
        let strips = array(root, "strips", context)?;
        let mut out = Vec::with_capacity(strips.len());
        for (si, s) in strips.iter().enumerate() {
            let ctx = format!("{context}: strips[{si}]");
            let s = s.as_dict().ok_or_else(|| Error::format(&ctx, "strip is not a dictionary"))?;
            let effects = match s.get("effects") {
                None => &[][..],
                Some(v) => v.as_array().ok_or_else(|| Error::format(&ctx, "effects is not an array"))?,
            };
            let mut list: Vec<Effect> = Vec::with_capacity(effects.len());
            for (ei, e) in effects.iter().enumerate() {
                let ectx = format!("{ctx}.effects[{ei}]");
                let e = e.as_dict().ok_or_else(|| Error::format(&ectx, "effect is not a dictionary"))?;
                let box_name = string(e, "displayname", &ectx)?;
                let ectx = format!("{ctx}.effects[{ei}] ({box_name})");
                let bypass = match e.get("bypass") {
                    None => false,
                    Some(Value::Bool(b)) => *b,
                    Some(Value::Integer(0)) => false,
                    Some(Value::Integer(1)) => true,
                    Some(o) => return Err(Error::format(&ectx, format!("bypass is {o:?}, not 0/1/bool"))),
                };
                let unit_d = dict(e, "unit", &ectx)?;
                let unit = unit_id(unit_d, &format!("{ectx}.unit"))?;
                let preset = AuPreset::from_dict(dict(e, "aupreset", &ectx)?, &format!("{ectx}.aupreset"))?;
                if preset.unit != unit {
                    return Err(Error::format(&ectx, format!("unit {unit} but its preset is for {}", preset.unit)));
                }
                if list.iter().any(|x| x.box_name == box_name) {
                    return Err(Error::format(&ectx, "box configured twice in one strip"));
                }
                list.push(Effect { box_name, bypass, unit, preset });
            }
            out.push(list);
        }
        Ok(AuStrip { strips: out })
    }

    pub fn effects(&self) -> impl Iterator<Item = &Effect> {
        self.strips.iter().flatten()
    }

    pub fn effect(&self, box_name: &str) -> Option<&Effect> {
        self.effects().find(|e| e.box_name == box_name)
    }
}

/// One property value from a `.propstrip`.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(tag = "type", content = "value")]
pub enum PropValue {
    UInt32(u32),
    Path(String),
    /// Any other representation key, kept as its plist kind and debug text.
    Other {
        key: String,
        repr: String,
    },
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Property {
    pub number: u32,
    pub value: PropValue,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct PropBox {
    pub name: String,
    pub properties: Vec<Property>,
}

impl PropBox {
    pub fn property(&self, number: u32) -> Option<&PropValue> {
        self.properties.iter().find(|p| p.number == number).map(|p| &p.value)
    }
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct PropStrip {
    pub boxes: Vec<PropBox>,
}

impl PropStrip {
    pub fn parse(bytes: &[u8], context: &str) -> Result<Self> {
        let (root, _) = plist::parse(bytes).map_err(|e| Error::plist(context, e))?;
        let root = root.as_dict().ok_or_else(|| Error::format(context, "root is not a dictionary"))?;
        let mut boxes: Vec<PropBox> = Vec::new();
        for (bi, b) in array(root, "Boxes", context)?.iter().enumerate() {
            let ctx = format!("{context}: Boxes[{bi}]");
            let b = b.as_dict().ok_or_else(|| Error::format(&ctx, "box is not a dictionary"))?;
            let name = string(b, "Name", &ctx)?;
            let mut properties: Vec<Property> = Vec::new();
            for (pi, p) in array(b, "Properties", &ctx)?.iter().enumerate() {
                let pctx = format!("{ctx} ({name}).Properties[{pi}]");
                let p = p.as_dict().ok_or_else(|| Error::format(&pctx, "property is not a dictionary"))?;
                let number = match p.get("Number") {
                    Some(Value::Integer(n)) if (0..=u32::MAX as i128).contains(n) => *n as u32,
                    Some(o) => return Err(Error::format(&pctx, format!("Number is {o:?}"))),
                    None => return Err(Error::missing(&pctx, "Number")),
                };
                let mut reprs = p.iter().filter(|(k, _)| *k != "Number");
                let (key, v) = reprs.next().ok_or_else(|| Error::format(&pctx, "property has no value"))?;
                if reprs.next().is_some() {
                    return Err(Error::format(&pctx, "property has more than one value"));
                }
                let value = match (key, v) {
                    ("UInt32", Value::Integer(n)) if (0..=u32::MAX as i128).contains(n) => PropValue::UInt32(*n as u32),
                    ("UInt32", o) => return Err(Error::format(&pctx, format!("UInt32 value {o:?}"))),
                    ("Path", Value::String(s)) => PropValue::Path(s.clone()),
                    ("Path", o) => return Err(Error::format(&pctx, format!("Path value is a {}", o.kind()))),
                    (k, o) => PropValue::Other { key: k.to_string(), repr: format!("{o:?}") },
                };
                if properties.iter().any(|q| q.number == number) {
                    return Err(Error::format(&pctx, format!("property {number} set twice")));
                }
                properties.push(Property { number, value });
            }
            if boxes.iter().any(|x| x.name == name) {
                return Err(Error::format(&ctx, format!("box {name} listed twice")));
            }
            boxes.push(PropBox { name, properties });
        }
        Ok(PropStrip { boxes })
    }

    pub fn get(&self, name: &str) -> Option<&PropBox> {
        self.boxes.iter().find(|b| b.name == name)
    }
}

pub(crate) fn dict<'a>(d: &'a Dict, key: &str, context: &str) -> Result<&'a Dict> {
    match d.get(key) {
        Some(Value::Dict(x)) => Ok(x),
        Some(o) => Err(Error::Type { context: context.into(), field: key.into(), expected: "dict", found: o.kind() }),
        None => Err(Error::missing(context, key)),
    }
}

pub(crate) fn array<'a>(d: &'a Dict, key: &str, context: &str) -> Result<&'a [Value]> {
    match d.get(key) {
        Some(Value::Array(x)) => Ok(x),
        Some(o) => Err(Error::Type { context: context.into(), field: key.into(), expected: "array", found: o.kind() }),
        None => Err(Error::missing(context, key)),
    }
}

pub(crate) fn string(d: &Dict, key: &str, context: &str) -> Result<String> {
    match d.get(key) {
        Some(Value::String(x)) => Ok(x.clone()),
        Some(o) => Err(Error::Type { context: context.into(), field: key.into(), expected: "string", found: o.kind() }),
        None => Err(Error::missing(context, key)),
    }
}
