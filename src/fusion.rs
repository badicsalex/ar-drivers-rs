use crate::{ARGlasses, Error, Fusion, GlassesEvent};
use nalgebra::{UnitQuaternion, Vector3};

type Result<T> = std::result::Result<T, Error>;

/*
complementary filter with very simple update algorithm:

(
    assuming:
    (S, S-) = current & previous estimated state
    d~S1 = rate sensor reading (e.g. gyro), high frequency, high drift, dead reckoning
    ~S2 = state sensor reading (e.g. grav/acc, mag), low frequency, high noise, low drift
    d_t1 = time elapse since last rate sensor sampling
)

S = S- + ratio * d~S1 * d_t1 + (1-ratio) * (~S2 - S-)
  = ratio * (S- + d~S1 * d_t1) + (1-ratio) * ~S2

this implies:

- the algorithm natively support state sensor(s) with different sampling frequency, incomplete reading,
or unreliable reading, as ~S2 variable is merely an optional correction
- the interpolation between ~S2 and the first term doesn't need to be linear or additive,
e.g. 3D angular interpolation is multiplicative
- the ratio can be adjusted based on quality & frequency of state sensor(s)

most glasses have acc & grav/acc readings in 1 bundle, but I prefer not using this assumption and still update them independently

 */
pub struct ComplementaryFilter {
    pub glasses: Box<dyn ARGlasses>,
    // estimation
    pub attitude: UnitQuaternion<f32>,
    // just old readings
    // prevAcc: (Vector3<f32>, u64),
    pub prev_gyro: (Vector3<f32>, u64),
    // prevMag: (Vector3<f32>, u64),
}

impl ComplementaryFilter {
    const BASE_GRAV_RATIO: f32 = 0.6;
    const BASE_MAG_RATIO: f32 = 0.5;

    const UP: Vector3<f32> = Vector3::new(0.0, 1.0, 0.0);
    const NORTH: Vector3<f32> = Vector3::new(1.0, 0.0, 0.0);

    pub fn new(glasses: Box<dyn ARGlasses>) -> Result<Self> {
        let attitude = UnitQuaternion::identity();
        let prev_gyro = (Vector3::zeros(), 0);
        let mut fusion = ComplementaryFilter {
            glasses,
            attitude,
            prev_gyro,
        };

        loop {
            // wait for the first non-zero gyro reading
            let next_event = fusion.next_event();
            match next_event {
                GlassesEvent::AccGyro {
                    accelerometer: _,
                    gyroscope,
                    timestamp,
                } => {
                    if gyroscope != Vector3::zeros() {
                        fusion.prev_gyro = (gyroscope, timestamp);
                        return Ok(fusion);
                    }
                }
                _ => {}
            }
        }
    }
    /// read until next valid event. Blocks.
    fn next_event(&mut self) -> GlassesEvent {
        loop {
            match self.glasses.read_event() {
                Ok(event) => return event,
                Err(e) => {
                    println!("Error reading event: {}", e);
                }
            }
        }
    }

    fn update_gyro(&mut self, gyro: Vector3<f32>, t: u64) -> () {
        let d_t1 = ((t - self.prev_gyro.1) as f32) / 1000.0;
        let d_S1_t1 = gyro * d_t1;

        let integrated =
            self.attitude * UnitQuaternion::from_euler_angles(d_S1_t1.y, d_S1_t1.x, d_S1_t1.z);

        self.attitude = integrated;
        self.prev_gyro = (gyro, t);
    }

    fn update_acc(&mut self, acc: Vector3<f32>, _t: u64) -> () {
        let uncorrected = self.attitude * ComplementaryFilter::UP;

        let delta = UnitQuaternion::rotation_between(&uncorrected, &acc);

        match delta {
            Option::Some(d) => {
                let corrected = self.attitude
                    * UnitQuaternion::nlerp(
                        &UnitQuaternion::identity(),
                        &d,
                        1.0 - ComplementaryFilter::BASE_GRAV_RATIO,
                    );

                self.attitude = corrected;
            }
            Option::None => {
                // no update
            }
        }
    }
}

impl Fusion for ComplementaryFilter {
    fn glasses(&mut self) -> &mut Box<dyn ARGlasses> {
        &mut self.glasses
    }

    fn attitude_quaternion(&self) -> UnitQuaternion<f32> {
        self.attitude
    }

    fn attitude_euler(&self) -> Vector3<f32> {
        let (roll, pitch, yaw) = self.attitude.euler_angles();
        Vector3::new(roll, pitch, yaw)
    }

    fn update(&mut self) -> () {
        let event = self.next_event();
        match event {
            GlassesEvent::AccGyro {
                accelerometer,
                gyroscope,
                timestamp,
            } => {
                self.update_gyro(gyroscope, timestamp);
                self.update_acc(accelerometer, timestamp);
            }
            _ => {
                // TODO: add magnetometer event etc
            }
        }
    }
}
