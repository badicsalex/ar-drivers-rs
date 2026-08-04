//! Relative mouse motion emitter.
//!
//! We accumulate *fractional* pixel movement over several IMU frames and emit integer pixel
//! deltas with `SendInput`, either every frame or coalesced to a fixed rate. Coalescing keeps
//! the number of injected events reasonable (a typical game handles 500 Hz fine) while the
//! accumulator preserves sub-pixel precision so movement stays smooth and accurate.

#[cfg(windows)]
mod platform {
    use windows_sys::Win32::UI::Input::KeyboardAndMouse::{
        INPUT, INPUT_MOUSE, MOUSEEVENTF_MOVE, MOUSEINPUT, SendInput,
    };

    /// Inject a relative mouse move of `(dx, dy)` pixels (right/up positive).
    pub fn emit_relative(dx: i32, dy: i32) {
        let input = INPUT {
            r#type: INPUT_MOUSE,
            Anonymous: windows_sys::Win32::UI::Input::KeyboardAndMouse::INPUT_0 {
                mi: MOUSEINPUT {
                    dx,
                    dy,
                    mouseData: 0,
                    dwFlags: MOUSEEVENTF_MOVE,
                    time: 0,
                    dwExtraInfo: 0,
                },
            },
        };
        // SAFETY: `input` is a valid, fully-initialized INPUT; we pass a pointer to a single
        // element and the count 1. SendInput is thread-safe for relative moves.
        unsafe {
            SendInput(1, &input, std::mem::size_of::<INPUT>() as i32);
        }
    }
}

#[cfg(not(windows))]
mod platform {
    /// No-op on non-Windows hosts (keeps tests runnable everywhere).
    pub fn emit_relative(_dx: i32, _dy: i32) {}
}

pub use platform::emit_relative;

/// Coalesces fractional pixel deltas into integer `SendInput` events.
#[derive(Debug, Clone)]
pub struct MouseEmitter {
    acc_x: f32,
    acc_y: f32,
    /// Seconds between emissions (0 = emit every frame).
    interval: f32,
    /// Wall-clock time accumulated since the last emission.
    since: f32,
}

impl MouseEmitter {
    /// Create an emitter. `emit_rate` of 0 means "emit on every frame".
    pub fn new(emit_rate: u32) -> Self {
        let interval = if emit_rate > 0 { 1.0 / emit_rate as f32 } else { 0.0 };
        Self { acc_x: 0.0, acc_y: 0.0, interval, since: 0.0 }
    }

    /// Add a fractional pixel delta to the accumulator.
    pub fn add(&mut self, dx: f32, dy: f32) {
        self.acc_x += dx;
        self.acc_y += dy;
    }

    /// Drop any pending fractional remainder (call when paused so it does not "catch up").
    pub fn clear(&mut self) {
        self.acc_x = 0.0;
        self.acc_y = 0.0;
        self.since = 0.0;
    }

    /// Call once per frame with the elapsed wall-clock time `dt` (seconds). Returns the integer
    /// `(dx, dy)` to emit now, or `(0, 0)` if it is not yet time to emit.
    pub fn tick(&mut self, dt: f32) -> (i32, i32) {
        if self.interval <= 0.0 {
            return self.take();
        }
        self.since += dt;
        if self.since >= self.interval {
            self.since = 0.0;
            self.take()
        } else {
            (0, 0)
        }
    }

    /// Round accumulated fractional pixels to integers, keeping the remainder.
    fn take(&mut self) -> (i32, i32) {
        let dx = self.acc_x.round() as i32;
        let dy = self.acc_y.round() as i32;
        self.acc_x -= dx as f32;
        self.acc_y -= dy as f32;
        (dx, dy)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn immediate_mode_emits_every_frame() {
        let mut e = MouseEmitter::new(0);
        e.add(0.6, 0.6);
        let (dx, dy) = e.tick(1.0 / 500.0);
        assert_eq!((dx, dy), (1, 1)); // rounds up, remainder kept
        assert!((e.acc_x + 0.4).abs() < 1e-6);
    }

    #[test]
    fn coalesced_mode_accumulates_then_emits() {
        let mut e = MouseEmitter::new(500); // 2 ms interval
        e.add(0.4, 0.0);
        assert_eq!(e.tick(0.001), (0, 0)); // not yet
        e.add(0.4, 0.0);
        assert_eq!(e.tick(0.001), (0, 0)); // 2 ms passed but only 0.8 accumulated
        e.add(0.3, 0.0);
        let (dx, _) = e.tick(0.001); // 3rd ms -> emit 1.1 -> 1 px
        assert_eq!(dx, 1);
    }
}
