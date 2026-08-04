//! Translate head orientation into relative mouse motion.
//!
//! The fusion filter gives us a body→world rotation. From it we derive the **heading**
//! (yaw, left/right) and **elevation** (pitch, up/down) of the head's forward direction.
//! Each frame we output the *delta* of those angles, scaled by sensitivity, optionally run
//! through a deadzone, an exponential smoother and a non-linear response curve. Sub-pixel
//! remainders are accumulated by the mouse emitter (see [`crate::mouse`]).
//!
//! Coordinate notes (the glasses use an Android-style RUB frame:
//! +X right, +Y up, +Z back; "turning left" is a positive rotation about +Y):
//!
//! * Forward direction (body) is `-Z`.
//! * `heading = atan2(-f.x, -f.z)` → increases when you look **right**.
//! * `elevation = asin(f.y)` → increases when you look **up**.
//! * Mouse X is `-Δheading * sensitivity` (look right → cursor right).
//! * Mouse Y is `-Δelevation * sensitivity` (look up → cursor up).

use nalgebra::{UnitQuaternion, Vector3};

use crate::config::Config;

/// Result of decomposing the current orientation into aim angles.
#[derive(Debug, Clone, Copy)]
pub struct HeadAngles {
    /// Yaw / left-right, radians. Increases when looking right.
    pub heading: f32,
    /// Pitch / up-down, radians. Increases when looking up.
    pub elevation: f32,
    /// True near the up/down poles where heading is unstable (gimbal lock).
    pub heading_unstable: bool,
}

/// Converts head angles into mouse deltas. Holds smoothing/accumulator state.
#[derive(Debug, Clone)]
pub struct HeadMapper {
    // Per-axis smoothed angular rates (rad/s).
    smooth_yaw: f32,
    smooth_pitch: f32,
    // Reference angles from the previous frame (for delta computation).
    prev_heading: f32,
    prev_elevation: f32,
    // Tunables (refreshed when the config epoch changes).
    sensitivity: f32,
    deadzone_dps: f32,
    vertical_lock: bool,
    invert_pitch: bool,
    invert_yaw: bool,
    accel_exponent: f32,
    accel_reference_dps: f32,
    max_speed_pps: f32,
}

impl HeadMapper {
    /// Create a mapper from the current configuration. `initial` provides the first reference
    /// angles so the first frame does not produce a jump.
    pub fn new(config: &Config, initial: HeadAngles) -> Self {
        Self {
            smooth_yaw: 0.0,
            smooth_pitch: 0.0,
            prev_heading: initial.heading,
            prev_elevation: initial.elevation,
            sensitivity: config.tracking.sensitivity,
            deadzone_dps: config.tracking.deadzone_dps,
            vertical_lock: config.tracking.vertical_lock,
            invert_pitch: config.tracking.invert_pitch,
            invert_yaw: config.tracking.invert_yaw,
            accel_exponent: config.tracking.accel_exponent.max(0.1),
            accel_reference_dps: config.tracking.accel_reference_dps.max(1.0),
            max_speed_pps: config.tracking.max_speed_pps.max(0.0),
        }
    }

    /// Refresh tunables from a new config snapshot (cheap, called on config epoch change).
    pub fn refresh(&mut self, config: &Config, current: HeadAngles) {
        self.sensitivity = config.tracking.sensitivity;
        self.deadzone_dps = config.tracking.deadzone_dps;
        self.vertical_lock = config.tracking.vertical_lock;
        self.invert_pitch = config.tracking.invert_pitch;
        self.invert_yaw = config.tracking.invert_yaw;
        self.accel_exponent = config.tracking.accel_exponent.max(0.1);
        self.accel_reference_dps = config.tracking.accel_reference_dps.max(1.0);
        self.max_speed_pps = config.tracking.max_speed_pps.max(0.0);
        // Keep the reference angles in sync so there is no jump after a reload.
        self.prev_heading = current.heading;
        self.prev_elevation = current.elevation;
        self.smooth_yaw = 0.0;
        self.smooth_pitch = 0.0;
    }

    /// Override the runtime vertical-lock flag (e.g. toggled from the tray menu).
    pub fn set_vertical_lock(&mut self, v: bool) {
        self.vertical_lock = v;
    }

    /// Re-sync reference angles to `current` without emitting anything. Used while paused or
    /// while tracking is (re)enabled so the first active frame starts from zero delta.
    pub fn resync(&mut self, current: HeadAngles) {
        self.prev_heading = current.heading;
        self.prev_elevation = current.elevation;
        self.smooth_yaw = 0.0;
        self.smooth_pitch = 0.0;
    }

    /// Decompose an orientation into aim angles.
    pub fn angles_of(orientation: UnitQuaternion<f32>) -> HeadAngles {
        let f = orientation * Vector3::new(0.0, 0.0, -1.0); // forward direction (world)
        let elevation = f.y.clamp(-1.0, 1.0).asin();
        let heading = f32::atan2(-f.x, -f.z);
        let heading_unstable = f.y.abs() > 0.999;
        HeadAngles { heading, elevation, heading_unstable }
    }

    /// Compute the mouse delta (in pixels, float) for one frame.
    ///
    /// * `current` — current head angles.
    /// * `dt`      — frame time in seconds.
    /// * `tau`     — smoothing time constant (seconds); 0 disables smoothing.
    ///
    /// Returns `(dx, dy)` where +x is right and +y is up.
    pub fn step(&mut self, current: HeadAngles, dt: f32, tau: f32) -> (f32, f32) {
        let dt = if dt > 0.0 { dt } else { 1.0 / 500.0 };

        // Delta angles, wrapped to (-π, π].
        let mut d_heading = current.heading - self.prev_heading;
        if d_heading > std::f32::consts::PI {
            d_heading -= 2.0 * std::f32::consts::PI;
        } else if d_heading < -std::f32::consts::PI {
            d_heading += 2.0 * std::f32::consts::PI;
        }
        let d_elevation = current.elevation - self.prev_elevation;

        // Angular rates (rad/s → deg/s for deadzone comparison).
        let yaw_rate = d_heading / dt;
        let pitch_rate = d_elevation / dt;

        // Soft deadzone: remove the deadzone amount but keep a continuous response.
        let yaw_rate = soft_deadzone(yaw_rate, self.deadzone_dps.to_radians());
        let pitch_rate = if self.vertical_lock {
            0.0
        } else {
            soft_deadzone(pitch_rate, self.deadzone_dps.to_radians())
        };

        // Exponential smoothing on the rate.
        if tau > 0.0 {
            let a = 1.0 - (-dt / tau).exp();
            self.smooth_yaw += (yaw_rate - self.smooth_yaw) * a;
            self.smooth_pitch += (pitch_rate - self.smooth_pitch) * a;
        } else {
            self.smooth_yaw = yaw_rate;
            self.smooth_pitch = pitch_rate;
        }

        // Convert rate (rad/s) → degrees/s, then to pixels/second via the non-linear curve.
        let yaw_deg_s = self.smooth_yaw * 180.0 / std::f32::consts::PI;
        let pitch_deg_s = self.smooth_pitch * 180.0 / std::f32::consts::PI;
        let yaw_pps = apply_curve(yaw_deg_s, self.sensitivity, self.accel_exponent, self.accel_reference_dps);
        let pitch_pps = apply_curve(pitch_deg_s, self.sensitivity, self.accel_exponent, self.accel_reference_dps);

        let (mut dx, mut dy) = (0.0f32, 0.0f32);
        if !current.heading_unstable {
            dx = -yaw_pps * dt;
        }
        dy = -pitch_pps * dt;

        // Clamp total speed.
        if self.max_speed_pps > 0.0 {
            let speed = (dx * dx + dy * dy).sqrt() / dt;
            if speed > self.max_speed_pps {
                let s = self.max_speed_pps / speed;
                dx *= s;
                dy *= s;
            }
        }

        if self.invert_yaw {
            dx = -dx;
        }
        if self.invert_pitch {
            dy = -dy;
        }

        self.prev_heading = current.heading;
        self.prev_elevation = current.elevation;
        (dx, dy)
    }
}

/// Soft (continuous) deadzone: `sign(r) * max(0, |r| - d)`.
fn soft_deadzone(rate: f32, deadzone: f32) -> f32 {
    let d = deadzone.abs();
    if rate > d {
        rate - d
    } else if rate < -d {
        rate + d
    } else {
        0.0
    }
}

/// Non-linear response curve. `exponent == 1` is linear; `> 1` is slower near center and
/// faster at the edges; `< 1` is the opposite.
fn apply_curve(rate_deg_s: f32, sensitivity: f32, exponent: f32, reference_dps: f32) -> f32 {
    let r = rate_deg_s / reference_dps;
    let shaped = r.signum() * r.abs().powf(exponent);
    shaped * reference_dps * sensitivity
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Config;

    #[test]
    fn deadzone_removes_small_motion() {
        assert_eq!(soft_deadzone(0.01, 0.05), 0.0);
        assert!((soft_deadzone(0.10, 0.05) - 0.05).abs() < 1e-6);
        assert!((soft_deadzone(-0.10, 0.05) + 0.05).abs() < 1e-6);
    }

    #[test]
    fn curve_linear_when_exponent_one() {
        let c = apply_curve(30.0, 10.0, 1.0, 45.0);
        assert!((c - 30.0 * 10.0).abs() < 1e-3, "c={}", c);
    }

    #[test]
    fn vertical_lock_zeroes_pitch() {
        let cfg = Config::default();
        let mut m = HeadMapper::new(&cfg, HeadAngles { heading: 0.0, elevation: 0.0, heading_unstable: false });
        // Force vertical lock via reflection-like path: rebuild with mutated config.
        let mut cfg2 = Config::default();
        cfg2.tracking.vertical_lock = true;
        m.refresh(&cfg2, HeadAngles { heading: 0.0, elevation: 0.0, heading_unstable: false });
        let (dx, dy) = m.step(HeadAngles { heading: 0.1, elevation: 0.1, heading_unstable: false }, 1.0 / 500.0, 0.0);
        assert!(dx != 0.0);
        assert_eq!(dy, 0.0);
    }
}
