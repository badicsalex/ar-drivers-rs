//! Minimal, dependency-free logger that writes timestamped lines to stderr and/or a file.
//!
//! This keeps the binary tiny and avoids pulling in `log`/`env_logger`. A single global
//! logger is created once at startup via [`init`].

use std::io::Write;
use std::sync::Mutex;
use std::time::{SystemTime, UNIX_EPOCH};

/// Logging verbosity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Level {
    /// Errors only.
    Error = 0,
    /// Warnings and errors.
    Warn = 1,
    /// Informational messages (default).
    Info = 2,
    /// Verbose diagnostics (enable with `--verbose`).
    Debug = 3,
}

struct Logger {
    level: Level,
    file: Option<Mutex<std::fs::File>>,
}

static mut LOGGER: Option<Logger> = None;
static LOGGER_INIT: std::sync::Once = std::sync::Once::new();

/// Initialize the global logger.
///
/// * `level` — minimum severity that will be printed.
/// * `file` — optional path. When `Some`, every line is also appended to this file
///   (used by the GUI/no-console build via `--log-file`).
pub fn init(level: Level, file: Option<std::path::PathBuf>) {
    LOGGER_INIT.call_once(|| {
        let file = file.and_then(|p| std::fs::OpenOptions::new().create(true).append(true).open(p).ok().map(Mutex::new));
        // SAFETY: call_once guarantees this runs exactly once, before any logging.
        unsafe {
            LOGGER = Some(Logger { level, file });
        }
    });
}

fn now_string() -> String {
    let d = SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default();
    let secs = d.as_secs() % 86400;
    let hh = secs / 3600;
    let mm = (secs % 3600) / 60;
    let ss = secs % 60;
    let ms = d.subsec_millis();
    format!("{hh:02}:{mm:02}:{ss:02}.{ms:03}")
}

/// Emit a log line if `level` is at least the configured threshold.
pub fn log(level: Level, msg: &str) {
    // SAFETY: LOGGER is fully initialized by init() which runs before main logic.
    let logger = unsafe { LOGGER.as_ref() };
    let Some(logger) = logger else { return };
    if level > logger.level {
        return;
    }
    let tag = match level {
        Level::Error => "ERROR",
        Level::Warn => "WARN ",
        Level::Info => "INFO ",
        Level::Debug => "DEBUG",
    };
    let line = format!("[{}] {} {}\n", now_string(), tag, msg);
    // stderr first (best-effort, never panic on a failed write).
    let _ = std::io::stderr().write_all(line.as_bytes());
    if let Some(f) = &logger.file {
        if let Ok(mut g) = f.lock() {
            let _ = g.write_all(line.as_bytes());
        }
    }
}

/// Log at [`Level::Error`].
#[macro_export]
macro_rules! log_error {
    ($($arg:tt)*) => { $crate::logger::log($crate::logger::Level::Error, &format!($($arg)*)); };
}

/// Log at [`Level::Warn`].
#[macro_export]
macro_rules! log_warn {
    ($($arg:tt)*) => { $crate::logger::log($crate::logger::Level::Warn, &format!($($arg)*)); };
}

/// Log at [`Level::Info`].
#[macro_export]
macro_rules! log_info {
    ($($arg:tt)*) => { $crate::logger::log($crate::logger::Level::Info, &format!($($arg)*)); };
}

/// Log at [`Level::Debug`].
#[macro_export]
macro_rules! log_debug {
    ($($arg:tt)*) => { $crate::logger::log($crate::logger::Level::Debug, &format!($($arg)*)); };
}
