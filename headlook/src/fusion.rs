//! Sensor fusion: estimate head orientation (a body→world rotation) from the glasses IMU.
//!
//! We implement two orientation filters on SO(3):
//!
//! * **Complementary / Mahony** — integrate gyroscope, then correct the tilt using the
//!   accelerometer as a gravity reference. An optional integral term slowly estimates the
//!   gyroscope bias. This is the default and gives a very stable, low-latency result.
//! * **Madgwick** — the same idea but the correction is a normalized gradient step whose
//!   size is set by `beta`. (Without a magnetometer we cannot observe absolute yaw, so yaw
//!   is free to drift slowly; this is harmless because the app only ever outputs *relative*
//!   head motion.)
//!
//! Gyroscope bias is also learned automatically while the head is still, which removes the
//! slow "cursor creep" you would otherwise get from a non-zero bias.

use nalgebra::{Quaternion, UnitQuaternion, Vector3, Zero};

use crate::config::{Config, FusionMode};

/// Orientation/attitude estimator.
#[derive(Debug, Clone)]
pub struct SensorFusion {
    /// Body→world rotation.
    orientation: UnitQuaternion<f32>,
    /// Learned gyroscope bias in the body frame (rad/s).
    bias: Vector3<f32>,
    /// Integral of the correction error (Mahony bias estimator), body frame (rad/s).
    integral_bias: Vector3<f32>,
    /// Active algorithm.
    mode: FusionMode,
    /// Complementary / Mahony gain (0..1).
    gain: f32,
    /// Madgwick beta.
    beta: f32,
    /// Whether to auto-learn gyro bias while still.
    auto_bias: bool,
    /// Seconds the head has been still (for bias learning + startup calibration).
    still_time: f32,
}

impl SensorFusion {
    /// Build a fusion filter from the configuration.
    pub fn new(config: &Config) -> Self {
        Self {
            orientation: UnitQuaternion::identity(),
            bias: Vector3::zeros(),
            integral_bias: Vector3::zeros(),
            mode: config.fusion_mode(),
            gain: config.filter.complementary_gain.clamp(0.0, 1.0),
            beta: config.filter.madgwick_beta.max(1e-4),
            auto_bias: config.filter.auto_gyro_bias,
            still_time: 0.0,
        }
    }

    /// Current estimated orientation (body→world).
    pub fn orientation(&self) -> UnitQuaternion<f32> {
        self.orientation
    }

    /// Current learned gyroscope bias (rad/s).
    pub fn bias(&self) -> Vector3<f32> {
        self.bias
    }

    /// Seconds the head has been (approximately) still.
    pub fn still_time(&self) -> f32 {
        self.still_time
    }

    /// Reset the filter to identity (no rotation) and clear the integral term.
    /// Keeps the learned bias.
    pub fn reset(&mut self) {
        self.orientation = UnitQuaternion::identity();
        self.integral_bias = Vector3::zeros();
        self.still_time = 0.0;
    }

    /// Update tunables from a new config without losing orientation or learned bias. Safe to
    /// call when the config epoch changes at runtime.
    pub fn refresh(&mut self, config: &Config) {
        self.mode = config.fusion_mode();
        self.gain = config.filter.complementary_gain.clamp(0.0, 1.0);
        self.beta = config.filter.madgwick_beta.max(1e-4);
        self.auto_bias = config.filter.auto_gyro_bias;
    }

    /// Initialize tilt from the accelerometer (used at startup for fast convergence).
    /// The device "feels" an acceleration opposite to gravity, so when the head is upright
    /// the accelerometer reads roughly `(0, 9.81, 0)`; the up direction in the body frame is
    /// therefore `accelerometer.normalize()`.
    pub fn reset_with_accel(&mut self, accel: Vector3<f32>) {
        let acc_norm = accel.norm();
        if acc_norm > 1e-3 {
            let up_meas = accel / acc_norm;
            let world_up = Vector3::new(0.0, 1.0, 0.0);
            if let Some(q) = UnitQuaternion::rotation_between(&world_up, &up_meas) {
                self.orientation = q;
            } else {
                // Anti-parallel (head upside-down): 180° flip about X.
                self.orientation = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f32::consts::PI);
            }
        }
        self.integral_bias = Vector3::zeros();
    }

    /// Feed one synchronized IMU sample.
    ///
    /// * `gyro_raw` — gyroscope reading in rad/s (body frame, from the glasses).
    /// * `accel`    — accelerometer reading in m/s² (body frame, from the glasses).
    /// * `dt`       — time since the previous sample, in seconds.
    pub fn update(&mut self, gyro_raw: Vector3<f32>, accel: Vector3<f32>, dt: f32) {
        let gyro = gyro_raw - self.bias;

        let acc_norm = accel.norm();
        if acc_norm > 1e-3 && dt > 0.0 {
            let u_meas = accel / acc_norm; // measured up direction in body frame
            let world_up = Vector3::new(0.0, 1.0, 0.0);
            // Estimated up direction in the body frame = Rᵀ · world_up.
            let u_est = self.orientation.inverse() * world_up;
            let e = u_meas.cross(&u_est); // error rotation axis (body frame)
            let e_mag = e.norm();
            if e_mag > 1e-6 {
                let corr = match self.mode {
                    FusionMode::Complementary => e * self.gain,
                    FusionMode::Madgwick => e * (self.beta / e_mag),
                };
                // Slow integral term (Mahony) used as an extra bias estimator.
                self.integral_bias += e * (self.gain * 0.2) * dt;
                let gyro_corrected = gyro + corr - self.integral_bias;
                self.integrate(gyro_corrected, dt);
            } else {
                self.integrate(gyro, dt);
            }
        } else {
            self.integrate(gyro, dt);
        }

        if self.auto_bias {
            // Still when gyro is near zero *after* bias removal and gravity magnitude is sane.
            let still = gyro.norm() < 0.05 && (acc_norm - 9.81).abs() < 1.0;
            if still {
                self.still_time += dt;
                // First-order approach toward the raw (≈0 when still) gyro, τ ≈ 2 s.
                let a = 1.0 - (-dt / 2.0).exp();
                self.bias += (gyro_raw - self.bias) * a;
            } else {
                self.still_time = 0.0;
            }
        }

        self.orientation.renormalize_fast();
    }

    /// Integrate the angular rate (body frame) over `dt` using a first-order quaternion update.
    fn integrate(&mut self, gyro: Vector3<f32>, dt: f32) {
        if dt <= 0.0 {
            return;
        }
        let half = 0.5 * dt;
        let delta_q = Quaternion::new(1.0, gyro.x * half, gyro.y * half, gyro.z * half);
        let q = *self.orientation.quaternion() * delta_q;
        self.orientation = UnitQuaternion::from_quaternion(q);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Config;

    fn approx(a: f32, b: f32, eps: f32) -> bool {
        (a - b).abs() < eps
    }

    #[test]
    fn yaw_rotation_produces_expected_heading() {
        let cfg = Config::default();
        let mut f = SensorFusion::new(&cfg);
        // Simulate a 90° left yaw (+Y). Forward (body -Z) should end up pointing -X (left).
        let dt = 1.0 / 200.0;
        for _ in 0..200 {
            f.update(Vector3::new(0.0, std::f32::consts::FRAC_PI_2, 0.0), Vector3::new(0.0, 9.81, 0.0), dt);
        }
        let fwd = f.orientation() * Vector3::new(0.0, 0.0, -1.0);
        assert!(approx(fwd.x, -1.0, 0.05), "forward.x={}", fwd.x);
        assert!(approx(fwd.z, 0.0, 0.05), "forward.z={}", fwd.z);
    }

    #[test]
    fn accel_initializes_tilt() {
        let cfg = Config::default();
        let mut f = SensorFusion::new(&cfg);
        // Head tilted forward 30°: gravity felt tilts back, i.e. accel gets a -Z component.
        f.reset_with_accel(Vector3::new(0.0, 9.81 * 30f32.to_radians().cos(), -9.81 * 30f32.to_radians().sin()));
        let fwd = f.orientation() * Vector3::new(0.0, 0.0, -1.0);
        assert!(fwd.y > 0.4, "expect forward to point up after tilt, fwd.y={}", fwd.y);
    }
}
