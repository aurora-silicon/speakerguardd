//! Errors and validation diagnostics.

use crate::plist::PlistError;
use serde::Serialize;
use std::fmt;

/// A hard failure: the input cannot be used.
#[derive(Clone, Debug, PartialEq)]
pub enum Error {
    Io {
        path: String,
        message: String,
    },
    Plist {
        context: String,
        source: PlistError,
    },
    /// Structurally wrong input.
    Format {
        context: String,
        message: String,
    },
    /// A required field is absent.
    Missing {
        context: String,
        field: String,
    },
    /// A field has the wrong type.
    Type {
        context: String,
        field: String,
        expected: &'static str,
        found: &'static str,
    },
    /// A field is outside its admissible range.
    Range {
        context: String,
        field: String,
        message: String,
    },
    /// The bundle's integrity record disagrees with its contents.
    Integrity {
        path: String,
        message: String,
    },
    /// The input uses a schema version this reader does not know.
    Version {
        context: String,
        message: String,
    },
}

pub type Result<T> = std::result::Result<T, Error>;

impl Error {
    pub fn format(context: &str, message: impl Into<String>) -> Self {
        Error::Format { context: context.into(), message: message.into() }
    }
    pub fn missing(context: &str, field: &str) -> Self {
        Error::Missing { context: context.into(), field: field.into() }
    }
    pub fn plist(context: &str, source: PlistError) -> Self {
        Error::Plist { context: context.into(), source }
    }
    pub fn range(context: &str, field: &str, message: impl Into<String>) -> Self {
        Error::Range { context: context.into(), field: field.into(), message: message.into() }
    }
    pub fn version(context: &str, message: impl Into<String>) -> Self {
        Error::Version { context: context.into(), message: message.into() }
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Io { path, message } => write!(f, "{path}: {message}"),
            Error::Plist { context, source } => write!(f, "{context}: {source}"),
            Error::Format { context, message } => write!(f, "{context}: {message}"),
            Error::Missing { context, field } => write!(f, "{context}: required field {field} is missing"),
            Error::Type { context, field, expected, found } => {
                write!(f, "{context}: {field} is a {found}, expected {expected}")
            }
            Error::Range { context, field, message } => write!(f, "{context}: {field} {message}"),
            Error::Integrity { path, message } => write!(f, "{path}: integrity check failed: {message}"),
            Error::Version { context, message } => write!(f, "{context}: unsupported version: {message}"),
        }
    }
}

impl std::error::Error for Error {}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Severity {
    /// Informational: something the engine may want to know.
    Info,
    /// Usable, but differs from what the reader knows (unknown keys, an
    /// unvalidated macOS build, a pending spec item).
    Warning,
    /// The bundle must not be used for protection.
    Error,
}

/// One validation finding.  `code` is stable and machine-matchable.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Diagnostic {
    pub severity: Severity,
    pub code: &'static str,
    pub context: String,
    pub message: String,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize)]
pub struct Diagnostics {
    pub items: Vec<Diagnostic>,
}

impl Diagnostics {
    pub fn push(
        &mut self,
        severity: Severity,
        code: &'static str,
        context: impl Into<String>,
        message: impl Into<String>,
    ) {
        self.items.push(Diagnostic { severity, code, context: context.into(), message: message.into() });
    }
    pub fn error(&mut self, code: &'static str, context: impl Into<String>, message: impl Into<String>) {
        self.push(Severity::Error, code, context, message)
    }
    pub fn warn(&mut self, code: &'static str, context: impl Into<String>, message: impl Into<String>) {
        self.push(Severity::Warning, code, context, message)
    }
    pub fn info(&mut self, code: &'static str, context: impl Into<String>, message: impl Into<String>) {
        self.push(Severity::Info, code, context, message)
    }
    pub fn from_error(&mut self, code: &'static str, e: &Error) {
        self.error(code, "", e.to_string())
    }
    pub fn has_errors(&self) -> bool {
        self.items.iter().any(|d| d.severity == Severity::Error)
    }
    pub fn count(&self, s: Severity) -> usize {
        self.items.iter().filter(|d| d.severity == s).count()
    }
    pub fn extend(&mut self, other: Diagnostics) {
        self.items.extend(other.items)
    }
}

impl fmt::Display for Diagnostic {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self.severity {
            Severity::Info => "info",
            Severity::Warning => "warning",
            Severity::Error => "error",
        };
        if self.context.is_empty() {
            write!(f, "{s}[{}]: {}", self.code, self.message)
        } else {
            write!(f, "{s}[{}] {}: {}", self.code, self.context, self.message)
        }
    }
}
