//! Configuration: loading, saving and validating `headlook.toml`.
//!
//! The config is parsed with `serde`/`toml`. Unknown top-level keys are rejected so typos are
//! caught early, but unknown *values* inside an enum (e.g. a bad `filter.mode`) are accepted by
//! serde and then normalized to a safe default at runtime.

use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

/// A key specification as written in the config, e.g. `"Ctrl"`, `"A"`, `"RMB"`, `"LMB"`.
pub type KeySpec = String;

/// Tracker thread priority.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "lowercase")]
pub enum ThreadPriority {
    /// `THREAD_PRIORITY_NORMAL`.
    Normal,
    /// `THREAD_PRIORITY_HIGHEST`.
    Above,
    /// `THREAD_PRIORITY_TIME_CRITICAL` (recommended).
    #[default]
    High,
    /// Same as `High` here, kept for user convenience.
    Realtime,
}

/// Tracking-related settings (sensitivity, deadzone, hotkey, pause keys).
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TrackingConfig {
    /// Global hotkey combo, e.g. `"Ctrl+A+R"`.
    pub hotkey: String,
    /// Start tracking automatically when the program launches.
    pub enabled_on_start: bool,
    /// Pixels per degree of head rotation.
    pub sensitivity: f32,
    /// Deadzone in degrees/second.
    pub deadzone_dps: f32,
    /// Only track yaw (left/right).
    pub vertical_lock: bool,
    /// Invert the pitch (up/down) axis.
    pub invert_pitch: bool,
    /// Invert the yaw (left/right) axis.
    pub invert_yaw: bool,
    /// Non-linear response exponent (1 = linear).
    pub accel_exponent: f32,
    /// Reference speed (deg/s) at which `accel_exponent` is evaluated.
    pub accel_reference_dps: f32,
    /// Max cursor speed in pixels/second (0 = unlimited).
    pub max_speed_pps: f32,
    /// Keys (besides LMB) that pause tracking while held.
    #[serde(default)]
    pub pause_keys: Vec<KeySpec>,
}

/// Sensor-fusion settings.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FilterConfig {
    /// Algorithm: `"complementary"` (a.k.a. Mahony) or `"madgwick"` (gradient descent).
    pub mode: String,
    /// Complementary/Mahony correction gain (0..1).
    pub complementary_gain: f32,
    /// Madgwick beta (gradient step size).
    pub madgwick_beta: f32,
    /// Output smoothing time constant (seconds). 0 disables smoothing.
    pub smoothing_tau: f32,
    /// Learn gyro bias automatically while the head is still.
    pub auto_gyro_bias: bool,
}

/// Mouse emission settings.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MouseConfig {
    /// Emission rate in Hz (coalescing). 0 = emit every IMU sample.
    pub emit_rate: u32,
    /// Audible beep on toggle.
    pub sound_feedback: bool,
}

/// System tray settings.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TrayConfig {
    /// Show a balloon notification when tracking toggles.
    pub notify_on_toggle: bool,
}

/// Advanced / internal tuning.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AdvancedConfig {
    /// Reconnect wait (seconds) after a disconnect.
    pub reconnect_seconds: f32,
    /// Seconds of stillness before (re)learning gyro bias.
    pub calibration_seconds: f32,
    /// Tracker thread priority.
    pub thread_priority: ThreadPriority,
    /// Number of consecutive read timeouts before declaring the device dead.
    pub timeout_count: u32,
}

/// The whole configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    /// Tracking section.
    pub tracking: TrackingConfig,
    /// Filter section.
    pub filter: FilterConfig,
    /// Mouse section.
    pub mouse: MouseConfig,
    /// Tray section.
    pub tray: TrayConfig,
    /// Advanced section.
    pub advanced: AdvancedConfig,
}

impl Default for Config {
    fn default() -> Self {
        // This MUST stay in sync with DEFAULT_CONFIG_TOML.
        Config {
            tracking: TrackingConfig {
                hotkey: "Ctrl+A+R".to_string(),
                enabled_on_start: true,
                sensitivity: 14.0,
                deadzone_dps: 1.5,
                vertical_lock: false,
                invert_pitch: false,
                invert_yaw: false,
                accel_exponent: 1.6,
                accel_reference_dps: 45.0,
                max_speed_pps: 0.0,
                pause_keys: vec!["RMB".to_string()],
            },
            filter: FilterConfig {
                mode: "complementary".to_string(),
                complementary_gain: 0.04,
                madgwick_beta: 0.1,
                smoothing_tau: 0.012,
                auto_gyro_bias: true,
            },
            mouse: MouseConfig {
                emit_rate: 500,
                sound_feedback: true,
            },
            tray: TrayConfig {
                notify_on_toggle: true,
            },
            advanced: AdvancedConfig {
                reconnect_seconds: 1.5,
                calibration_seconds: 1.0,
                thread_priority: ThreadPriority::High,
                timeout_count: 40,
            },
        }
    }
}

/// The default configuration, used the first time the program runs (and as documentation).
pub const DEFAULT_CONFIG_TOML: &str = r#"# HeadLook configuration file.
# Edit values to taste, save, then "Reload config" from the tray menu (or just toggle).
# Runtime-safe values (sensitivity, deadzone, hotkey, pause_keys, smoothing, ...) are
# picked up automatically within ~2 seconds.

[tracking]
hotkey = "Ctrl+A+R"
enabled_on_start = true
sensitivity = 14.0
deadzone_dps = 1.5
vertical_lock = false
invert_pitch = false
invert_yaw = false
accel_exponent = 1.6
accel_reference_dps = 45.0
max_speed_pps = 0.0
pause_keys = ["RMB"]

[filter]
mode = "complementary"
complementary_gain = 0.04
madgwick_beta = 0.1
smoothing_tau = 0.012
auto_gyro_bias = true

[mouse]
emit_rate = 500
sound_feedback = true

[tray]
notify_on_toggle = true

[advanced]
reconnect_seconds = 1.5
calibration_seconds = 1.0
thread_priority = "high"
timeout_count = 40
"#;

impl Config {
    /// Parse from TOML text.
    pub fn from_toml(text: &str) -> Result<Self, String> {
        toml::from_str(text).map_err(|e| format!("config parse error: {e}"))
    }

    /// Serialize back to TOML (used when writing the default file).
    pub fn to_toml(&self) -> Result<String, String> {
        toml::to_string_pretty(self).map_err(|e| format!("config serialize error: {e}"))
    }

    /// Clamp obviously invalid settings to safe ranges and return a list of warnings.
    pub fn sanitize(&mut self) -> Vec<String> {
        let mut w = Vec::new();
        if self.tracking.sensitivity <= 0.0 {
            w.push("tracking.sensitivity must be > 0; resetting to 14".into());
            self.tracking.sensitivity = 14.0;
        }
        if self.tracking.deadzone_dps < 0.0 {
            self.tracking.deadzone_dps = 0.0;
        }
        if self.tracking.accel_exponent < 0.1 {
            self.tracking.accel_exponent = 0.1;
        }
        if self.filter.complementary_gain < 0.0 || self.filter.complementary_gain > 1.0 {
            w.push("filter.complementary_gain should be in [0,1]; clamping".into());
            self.filter.complementary_gain = self.filter.complementary_gain.clamp(0.0, 1.0);
        }
        if self.filter.smoothing_tau < 0.0 {
            self.filter.smoothing_tau = 0.0;
        }
        if self.mouse.emit_rate > 2000 {
            w.push("mouse.emit_rate unusually high; clamping to 2000".into());
            self.mouse.emit_rate = 2000;
        }
        w
    }

    /// The selected fusion algorithm (normalized; unknown strings fall back to complementary).
    pub fn fusion_mode(&self) -> FusionMode {
        match self.filter.mode.to_ascii_lowercase().as_str() {
            "madgwick" => FusionMode::Madgwick,
            _ => FusionMode::Complementary,
        }
    }
}

/// Which fusion algorithm is active.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FusionMode {
    /// Complementary / Mahony SO(3) filter with an optional integral bias term.
    Complementary,
    /// Madgwick gradient-descent filter.
    Madgwick,
}

/// Resolve the config file location: explicit path, next to the exe, or `%APPDATA%/HeadLook`.
pub fn config_path(explicit: Option<&Path>) -> PathBuf {
    if let Some(p) = explicit {
        return p.to_path_buf();
    }
    if let Ok(exe) = std::env::current_exe() {
        if let Some(dir) = exe.parent() {
            let p = dir.join("headlook.toml");
            if p.exists() {
                return p;
            }
        }
    }
    if let Ok(appdata) = std::env::var("APPDATA") {
        let p = PathBuf::from(appdata).join("HeadLook").join("headlook.toml");
        if p.exists() {
            return p;
        }
    }
    // Fall back to next to the exe whether or not it exists yet.
    std::env::current_exe()
        .ok()
        .and_then(|e| e.parent().map(|d| d.join("headlook.toml")))
        .unwrap_or_else(|| PathBuf::from("headlook.toml"))
}

/// Load config from `path`, creating a default file if it does not exist.
pub fn load_or_create(path: &Path, verbose: bool) -> Config {
    if path.exists() {
        match std::fs::read_to_string(path) {
            Ok(text) => match Config::from_toml(&text) {
                Ok(mut cfg) => {
                    let warns = cfg.sanitize();
                    for w in &warns {
                        log_warn!("{}", w);
                    }
                    if verbose {
                        log_info!("Loaded config from {}", path.display());
                    }
                    return cfg;
                }
                Err(e) => {
                    log_error!("{} — falling back to defaults", e);
                }
            },
            Err(e) => {
                log_error!("Cannot read {}: {} — falling back to defaults", path.display(), e);
            }
        }
    } else {
        log_info!("No config at {} — writing defaults", path.display());
        if let Some(dir) = path.parent() {
            let _ = std::fs::create_dir_all(dir);
        }
        if let Err(e) = std::fs::write(path, DEFAULT_CONFIG_TOML) {
            log_warn!("Could not write default config: {}", e);
        }
    }
    let mut cfg = Config::default();
    cfg.sanitize();
    cfg
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_toml_is_valid_and_matches_defaults() {
        let parsed = Config::from_toml(DEFAULT_CONFIG_TOML).expect("default TOML must parse");
        let def = Config::default();
        // They should be equivalent after a round-trip.
        assert_eq!(parsed.to_toml().unwrap(), def.to_toml().unwrap());
        assert_eq!(parsed.tracking.hotkey, "Ctrl+A+R");
        assert_eq!(parsed.tracking.pause_keys, vec!["RMB".to_string()]);
        assert!(parsed.mouse.emit_rate > 0);
    }

    #[test]
    fn unknown_top_level_key_is_rejected() {
        let bad = r#"
[tracking]
hotkey = "A"
[bogus]
x = 1
"#;
        assert!(Config::from_toml(bad).is_err());
    }
}
