//! Low-level keyboard / mouse-button polling via `GetAsyncKeyState`.
//!
//! Polling (rather than a low-level hook) keeps latency minimal and avoids the extra message
//! loop a hook would require. The trade-off: keys used by the hotkey/combo are *not* swallowed,
//! so they can still reach the game. For a head-look app that is acceptable and even desirable
//! (the hotkey should not be intercepted).

#[cfg(windows)]
mod platform {
    use windows_sys::Win32::UI::Input::KeyboardAndMouse::GetAsyncKeyState;

    /// Returns true if the given virtual key is currently pressed. `vk` uses the same codes as
    /// `VK_*` in the Windows API.
    pub fn key_down(vk: i32) -> bool {
        // SAFETY: GetAsyncKeyState takes a single virtual-key code (0..0xFF). We only pass valid
        // codes produced by [`vk_of_name`].
        let state = unsafe { GetAsyncKeyState(vk) };
        (state as u16 & 0x8000u16) != 0
    }
}

#[cfg(not(windows))]
mod platform {
    /// No-op on non-Windows hosts.
    pub fn key_down(_vk: i32) -> bool {
        false
    }
}

pub use platform::key_down;

/// Resolve a human-friendly key name to a Windows virtual-key code.
pub fn vk_of_name(name: &str) -> Option<i32> {
    let n = name.trim();
    if let Some(hex) = n.strip_prefix("0x").or_else(|| n.strip_prefix("0X")) {
        return i32::from_str_radix(hex, 16).ok();
    }
    let lower = n.to_ascii_lowercase();
    let v = match lower.as_str() {
        "lmb" | "mouse1" | "mouse_left" => 0x01,
        "rmb" | "mouse2" | "mouse_right" => 0x02,
        "mmb" | "mouse3" | "mouse_middle" => 0x04,
        "x1" | "mouse4" | "mouse_x1" => 0x05,
        "x2" | "mouse5" | "mouse_x2" => 0x06,
        "back" | "backspace" => 0x08,
        "tab" => 0x09,
        "enter" | "return" => 0x0D,
        "shift" => 0x10,
        "ctrl" | "control" => 0x11,
        "alt" | "menu" => 0x12,
        "pause" => 0x13,
        "caps" | "capslock" => 0x14,
        "esc" | "escape" => 0x1B,
        "space" | " " => 0x20,
        "pgup" | "pageup" => 0x21,
        "pgdn" | "pagedown" => 0x22,
        "end" => 0x23,
        "home" => 0x24,
        "left" | "arrowleft" => 0x25,
        "up" | "arrowup" => 0x26,
        "right" | "arrowright" => 0x27,
        "down" | "arrowdown" => 0x28,
        "ins" | "insert" => 0x2D,
        "del" | "delete" => 0x2E,
        "lwin" | "win" => 0x5B,
        "rwin" => 0x5C,
        _ => {
            // Digits 0-9 and letters a-z. Single-char uppercase letters map to their VK code.
            if lower.len() == 1 {
                let c = lower.chars().next().unwrap();
                if c.is_ascii_digit() {
                    return Some(0x30 + (c as i32 - '0' as i32));
                }
                if c.is_ascii_lowercase() {
                    return Some(0x41 + (c as i32 - 'a' as i32));
                }
            }
            // Function keys F1..F24.
            if let Some(rest) = lower.strip_prefix('f') {
                if let Ok(n) = rest.parse::<i32>() {
                    if (1..=24).contains(&n) {
                        return Some(0x70 + (n - 1));
                    }
                }
            }
            return None;
        }
    };
    Some(v)
}

/// Parse a combo string like `"Ctrl+A+R"` into a list of virtual-key codes.
pub fn parse_combo(spec: &str) -> Result<Vec<i32>, String> {
    let mut vks = Vec::new();
    for part in spec.split('+') {
        let name = part.trim();
        if name.is_empty() {
            continue;
        }
        match vk_of_name(name) {
            Some(vk) => vks.push(vk),
            None => return Err(format!("unknown key in hotkey/combo: '{name}'")),
        }
    }
    if vks.is_empty() {
        return Err("hotkey is empty".to_string());
    }
    Ok(vks)
}

/// Parse the `pause_keys` list; unrecognized keys are warned about and dropped (LMB always
/// pauses regardless of this list).
pub fn parse_pause_keys(specs: &[String]) -> Vec<i32> {
    let mut vks = Vec::new();
    for s in specs {
        match vk_of_name(s) {
            Some(vk) => vks.push(vk),
            None => crate::log_warn!("Ignoring unknown pause key '{}'", s),
        }
    }
    vks
}

/// True when the left mouse button (or any of `extra` keys) is currently held.
pub fn is_paused(extra: &[i32]) -> bool {
    if key_down(0x01) {
        return true; // LMB always pauses
    }
    extra.iter().any(|&vk| key_down(vk))
}

/// Edge-detecting watcher for a key combo (e.g. the toggle hotkey).
#[derive(Debug, Clone)]
pub struct HotkeyWatcher {
    keys: Vec<i32>,
    prev: bool,
}

impl HotkeyWatcher {
    /// Build from a parsed combo.
    pub fn new(keys: Vec<i32>) -> Self {
        Self { keys, prev: false }
    }

    /// Poll once. Returns `true` exactly once, on the rising edge where all keys are down
    /// (i.e. the first frame where the full combo is held).
    pub fn poll_edge(&mut self) -> bool {
        let all = !self.keys.is_empty() && self.keys.iter().all(|&k| key_down(k));
        let edge = all && !self.prev;
        self.prev = all;
        edge
    }
}
