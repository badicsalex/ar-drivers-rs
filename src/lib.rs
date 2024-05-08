// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

#![warn(missing_docs)]
//! This crate contains a simplified driver for Rokid Air and Mad Gaze Glow AR glasses.
//! It supports getting basic sensor data and setting up the display.
//!
//! Example usage (in a thread, probably):
//! ```ignore
//! let mut glasses = any_glasses().unwrap();
//! loop {
//!     match glasses.read_event().unwrap() {
//!         GlassesEvent::AccGyro {accelerometer, gyroscope} => ...,
//!         GlassesEvent::Magnetometer(data) => ...,
//!         GlassesEvent::KeyPress(data) => ...,
//!         _ => {}
//!     }
//! }
//! ```
//!
//! As opposed to e.g. Rokid's own API, this is all that you get, since this is what comes
//! out of the hardware. To get quaternions, you should probably use a lib that implements
//! Madgwicks algorithm or a proper EKF. One good choice is the `eskf` crate.
//!
//! ## Feature flags
//!
//! Support for individual AR glasses types ca be enabled with the following features:
//!
//! * `mad_gaze`: Mad Gaze Glow
//! * `nreal`: Nreal Light
//! * `rokid`: Rokid Air
//!
//! All of them are enabled by default, which may bring in some unwanted dependencies if you
//! only want to support a specific type.

use nalgebra::{Isometry3, Matrix3, UnitQuaternion, Vector2, Vector3};

use crate::naive_cf::NaiveCF;

#[cfg(feature = "grawoow")]
pub mod grawoow;
#[cfg(feature = "mad_gaze")]
pub mod mad_gaze;
mod naive_cf;
#[cfg(feature = "nreal")]
pub mod nreal_air;
#[cfg(feature = "nreal")]
pub mod nreal_light;
#[cfg(feature = "rokid")]
pub mod rokid;
mod util;

/// Possible errors resulting from `ar-drivers` API calls
#[derive(Debug)]
pub enum Error {
    /// A standard IO error happened. See [`std::io::Error`] for specifics
    IoError(std::io::Error),
    /// An rusb error happened. See [`rusb::Error`] for specifics
    #[cfg(feature = "rusb")]
    UsbError(rusb::Error),
    /// A hidapi error happened. See [`hidapi::HidError`] for specifics
    #[cfg(feature = "hidapi")]
    HidError(hidapi::HidError),
    /// A serialport error happened. See [`serialport::Error`] for specifics
    #[cfg(feature = "serialport")]
    SerialPortError(serialport::Error),
    /// No glasses were found.
    NotFound,
    /// The feature is not available with this headset.
    NotImplemented,
    /// Packet sending or reception timed out. Note that this is not the only
    /// timeout error that is sent (e.g. UsbError can contain a timeout), and
    /// also this is usually a fatal one.
    PacketTimeout,
    /// Other fatal error, usually a problem with the library itself, or
    /// a device support issue. File a bug if you encounter this.
    Other(&'static str),
}

type Result<T> = std::result::Result<T, Error>;

/*
high level interface of glasses & state estimation, with the following built-in fusion pipeline:

(the first version should only use complementary filter for simplicity and sanity test)

- roll/pitch <= acc + gyro (complementary filter)
  - assuming that acc vector always pointed up, spacecraft moving in that direction can create 1G artificial gravity
    - TODO: this obviously assumes no steadily accelerating frame, at which point up d_acc has to be used for correction
  - TODO: use ESKF (error-state/multiplicatory KF, https://arxiv.org/abs/1711.02508)
- gyro-yaw <= gyro (integrate over time)
- mag-yaw <= mag + roll/pitch (arctan)
  - TODO: mag calibration?
     (continuous ellipsoid fitting, assuming homogeneous E-M environment & hardpoint-mounted E-M interference)
- yaw <= mag-yaw + gyro-gyro (complementary filter)
  - TODO: use EKF

CAUTION: unlike [[GlassesEvent]], all states & outputs should use FRD reference frame
 (forward, right, down, corresponding to roll, pitch, yaw in Euler angles-represented rotation)

FRD is the standard frame for aerospace, and is also the default frame for NALgebra
*/
pub trait Fusion: Send {
    fn glasses(&mut self) -> &mut Box<dyn ARGlasses>;
    // TODO: only declared mutable as many API of ARGlasses are also mutable

    /// primary estimation output
    /// can be used to convert to Euler angles of different conventions
    fn attitude_quaternion(&self) -> UnitQuaternion<f32>;

    /// use FRD frame as error in Quaternion is multiplicative & is over-defined
    fn inconsistency(&self) -> f32;

    fn update(&mut self) -> ();
}

impl dyn Fusion {
    pub fn attitude_frd_rad(&self) -> Vector3<f32> {
        let (roll, pitch, yaw) = (self.attitude_quaternion()).euler_angles();
        Vector3::new(roll, pitch, yaw)
    }

    pub fn attitude_frd_deg(&self) -> Vector3<f32> {
        self.attitude_frd_rad().map(|x| x.to_degrees())
    }
}

pub fn any_fusion() -> Result<Box<dyn Fusion>> {
    // let glasses = any_glasses()?;
    let glasses = any_glasses()?;
    Ok(Box::new(NaiveCF::new(glasses)?))
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::IoError(e) => Some(e),
            #[cfg(feature = "rusb")]
            Error::UsbError(e) => Some(e),
            #[cfg(feature = "hidapi")]
            Error::HidError(e) => Some(e),
            #[cfg(feature = "serialport")]
            Error::SerialPortError(e) => Some(e),
            _ => None,
        }
    }
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Error::IoError(_) => "I/O error",
            #[cfg(feature = "rusb")]
            Error::UsbError(_) => "Libusb error",
            #[cfg(feature = "hidapi")]
            Error::HidError(_) => "Hidapi error",
            #[cfg(feature = "serialport")]
            Error::SerialPortError(_) => "Serial error",
            Error::NotFound => "Glasses not found",
            Error::NotImplemented => "Not implemented for these glasses",
            Error::PacketTimeout => "Packet timeout",
            Error::Other(s) => s,
        })
    }
}

/// AR glasses sensor event, got from [`ARGlasses::read_event`]
///
/// Coordinate system is "RUB": Positive X is Right, Positive Y is Up, Positive Z is backwards.
/// This is the same as the Android sensor coordinate system.
#[derive(Debug, Clone)]
pub enum GlassesEvent {
    /// Synchronized accelerometer and gyroscope data.
    AccGyro {
        /// Accelerometer data in m^2/s.
        ///
        /// Remember that while gravitational acceleration is "down", the acceleration
        /// the device "feels" is the one opposite from that, so the normal reading
        /// when the device is upright is (0, 9.81, 0)
        accelerometer: Vector3<f32>,
        /// Gyroscope data. Right handed rotation in rad/sec,
        /// e.g. turning left is positive y axis.
        gyroscope: Vector3<f32>,
        /// Timestamp, in device time, in microseconds
        timestamp: u64,
    },
    /// Magnetometer data.
    Magnetometer {
        /// Direction of magnetic north (more or less). Unit is uT.
        magnetometer: Vector3<f32>,
        /// Timestamp, in device time, in microseconds
        timestamp: u64,
    },
    /// A key was pressed (sent once per press)
    /// The number is a key ID, starting from 0.
    KeyPress(u8),

    /// Proximity sensor senses the user, i.e. the glasses were put on
    /// Sent once per event.
    ProximityNear,

    /// Proximity sensor senses the user, i.e. the glasses were taken off.
    /// Sent once per event.
    ProximityFar,
    /// Ambient light level. Unit is vendor-specific
    AmbientLight(u16),
    /// V-sync happened on the device
    VSync,
}

/// Display mode used by [`ARGlasses::set_display_mode`]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DisplayMode {
    /// Picture should be same for both eyes (simple full HD mode)
    SameOnBoth,
    /// Set display to 3840*1080 or 3840x1200,
    /// where the left half is the left eye, the right half is the right eye
    Stereo,
    /// Set display to half-SBS mode, which presents itself as 1920*1080 resolution,
    /// but actually scales down everything to 960x540,then upscales to 3840x1080
    HalfSBS,
    /// Set display to mirrored high refresh rate mode (typically 120Hz)
    HighRefreshRate,
    /// Set display to high refresh rate SBS mode
    HighRefreshRateSBS,
}

/// Display side used by [`ARGlasses::view_matrix`]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Side {
    /// Left display
    Left,
    /// Right display
    Right,
}

/// Common interface for AR implemented glasses
pub trait ARGlasses: Send {
    /// Get the serial number of the glasses
    fn serial(&mut self) -> Result<String>;
    /// Get a single sensor event. Blocks.
    fn read_event(&mut self) -> Result<GlassesEvent>;
    /// Get the display mode of the glasses. See [`DisplayMode`]
    fn get_display_mode(&mut self) -> Result<DisplayMode>;
    /// Set the display mode of the glasses. See [`DisplayMode`]
    fn set_display_mode(&mut self, display_mode: DisplayMode) -> Result<()>;
    /// Field of view of the display along the horizontal axis, in radians
    fn display_fov(&self) -> f32;
    /// Transformation from IMU frame to display frame, at the specified
    /// IPD (interpupillary distance). The `ipd` parameter is in meters.
    /// A typical value is 0.07.
    fn imu_to_display_matrix(&self, side: Side, ipd: f32) -> Isometry3<f64>;
    /// Name of the device
    fn name(&self) -> &'static str;
    /// Get built-in camera descriptors
    fn cameras(&self) -> Result<Vec<CameraDescriptor>> {
        Ok(Vec::new())
    }
    /// Get the available display matrices
    fn display_matrices(&self) -> Result<(DisplayMatrices, DisplayMatrices)> {
        Err(Error::NotImplemented)
    }
    /// The additional delay (in usecs) of the glasses' display from getting the data
    /// on DisplayPort. This is not really an absolute value, but more of
    /// a relative measure between different glasses.
    /// In the future this may depend on the current display mode.
    fn display_delay(&self) -> u64;
}

/// Represents one built-in camera
///
/// Warning: Experimental. May change between any versions.
#[derive(Debug, Clone)]
pub struct CameraDescriptor {
    /// The unique name for the type of the camera.
    pub name: &'static str,
    /// The width and height in pixels for the calibration data
    pub resolution: Vector2<f64>,
    /// The intrinsic matrix of the camera
    pub intrinsic_matrix: Matrix3<f64>,
    /// Distortion coefficients: k1, k2, p1, p2, k3
    pub distortion: [f64; 5],
    /// Additional rectification matrix for stereo cameras
    pub stereo_rotation: UnitQuaternion<f64>,
    /// Transformation from the IMU frame to the camera frame
    pub imu_to_camera: Isometry3<f64>,
}

/// Represents the decomposed extrinsic and intrinsic matrices for the display's lens.
#[derive(Debug, Clone)]
pub struct DisplayMatrices {
    /// The intrinsic matrix of the display
    pub intrinsic_matrix: Matrix3<f64>,
    /// Native resolution of the display, in pixels.
    pub resolution: (u32, u32),
    /// Extrinsic matrix of the display; translation in meters.
    pub isometry: Isometry3<f64>,
}

fn upcast<G: ARGlasses + 'static>(result: Result<G>) -> Result<Box<dyn ARGlasses>> {
    result.map(|glasses| Box::new(glasses) as Box<dyn ARGlasses>)
}

/// Convenience function to detect and connect to any of the supported glasses
#[cfg(not(target_os = "android"))]
pub fn any_glasses() -> Result<Box<dyn ARGlasses>> {
    let glasses_factories: Vec<(&str, fn() -> Result<Box<dyn ARGlasses>>)> = vec![
        #[cfg(feature = "rokid")]
        ("RokidAir", || upcast(rokid::RokidAir::new())),
        #[cfg(feature = "nreal")]
        ("NrealAir", || upcast(nreal_air::NrealAir::new())),
        #[cfg(feature = "nreal")]
        ("NrealLight", || upcast(nreal_light::NrealLight::new())),
        #[cfg(feature = "grawoow")]
        ("GrawoowG530", || upcast(grawoow::GrawoowG530::new())),
        #[cfg(feature = "mad_gaze")]
        ("MadGazeGlow", || upcast(mad_gaze::MadGazeGlow::new())),
    ];

    glasses_factories
        .into_iter()
        .find_map(|(glasses_type, factory)| {
            let factory: fn() -> Result<Box<dyn ARGlasses>> = factory;

            factory()
                .map_err(|e| {
                    //
                    println!("can't find {}: {}", glasses_type, e)
                })
                .ok()
                .map(|v| {
                    println!("found {}", glasses_type);
                    v
                })
        })
        .ok_or(Error::NotFound)
}

impl From<std::io::Error> for Error {
    fn from(e: std::io::Error) -> Self {
        Error::IoError(e)
    }
}

#[cfg(feature = "rusb")]
impl From<rusb::Error> for Error {
    fn from(e: rusb::Error) -> Self {
        Error::UsbError(e)
    }
}

#[cfg(feature = "hidapi")]
impl From<hidapi::HidError> for Error {
    fn from(e: hidapi::HidError) -> Self {
        Error::HidError(e)
    }
}

#[cfg(feature = "serialport")]
impl From<serialport::Error> for Error {
    fn from(e: serialport::Error) -> Self {
        Error::SerialPortError(e)
    }
}

impl From<&'static str> for Error {
    fn from(e: &'static str) -> Self {
        Error::Other(e)
    }
}
