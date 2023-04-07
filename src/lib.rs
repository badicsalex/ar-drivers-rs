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
//!         GlassesEvent::Accelerometer(data) => ...,
//!         GlassesEvent::Gyroscope(data) => ...,
//!         GlassesEvent::Magnetometer(data) => ...,
//!         GlassesEvent::Misc(data) => ...,
//!     }
//! }
//! ```
//!
//! As opposed to e.g. Rokid's own API, this is all that you get, since this is what comes
//! out of the hardware. To get quaternions, you should probably use a lib that implements
//! Madgwicks algorithm or a proper EKF. One good choice is the `eskf` crate.

#[cfg(feature = "mad_gaze")]
use mad_gaze::MadGazeGlow;
#[cfg(feature = "nreal")]
use nreal::NrealLight;
#[cfg(feature = "rokid")]
use rokid::RokidAir;

#[cfg(feature = "mad_gaze")]
pub mod mad_gaze;
#[cfg(feature = "nreal")]
pub mod nreal;
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
    #[cfg(feature = "rusb")]
    HidError(hidapi::HidError),
    /// A serialport error happened. See [`serialport::Error`] for specifics
    #[cfg(feature = "serialport")]
    SerialPortError(serialport::Error),
    /// No glasses were found.
    NotFound,
    /// Packet sending or reception timed out. Note that this is not the only
    /// timeout error that is sent (e.g. UsbError can contain a timeout), and
    /// also this is usually a fatal one.
    PacketTimeout,
    /// The glasses (or one of its devices) were disconnected
    Disconnected(&'static str),
    /// Other fatal error, usually a problem with the library itself, or
    /// a device support issue. File a bug if you encounter this.
    Other(&'static str),
}

type Result<T> = std::result::Result<T, Error>;

/// AR glasses sensor event, got from [`Glasses::read_event`]
#[derive(Debug, Clone)]
pub enum GlassesEvent {
    /// Synchronized accelerometer and gyroscope data.
    /// The timestamps are guaranteed to be the same.
    AccGyro {
        /// Accelerometer data in m^2/s
        accelerometer: SensorData3D,
        /// Gyroscope data. Right handed rotation in rad/sec,
        /// e.g. turning left is positive y axis.
        gyroscope: SensorData3D,
    },
    /// Magnetometer data. Vector points to magnetic north (mostly)
    Magnetometer(SensorData3D),
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

/// Structure to hold a typical 3D sensor value (accelerometer, gyroscope, magnetomer)
#[derive(Debug, Clone)]
pub struct SensorData3D {
    /// Timestamp, in device time, in microseconds
    pub timestamp: u64,
    /// Sensor value in the X coordinate. Positive is Right.
    pub x: f32,
    /// Sensor value in the Y coordinate. Positive is Up.
    pub y: f32,
    /// Sensor value in the Z coordinate. Positive is Backwards.
    pub z: f32,
}

/// Display mode used by [`Glasses::set_display_mode`]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DisplayMode {
    /// Picture should be same for both eyes (simple full HD mode)
    SameOnBoth = 0,
    /// Set display to 3840*1080, where the left half is the left eye, the right half
    /// is the right eye
    Stereo = 1,
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
    /// Tilt of the display compared to the IMU readings, in radians. Positive means tilted up.
    fn display_tilt(&self) -> f32;
    /// Name of the device
    fn name(&self) -> &'static str;
}

/// Convenience function to detect and connect to any of the supported glasses
pub fn any_glasses() -> Result<Box<dyn ARGlasses>> {
    #[cfg(feature = "rokid")]
    if let Ok(glasses) = RokidAir::new() {
        return Ok(Box::new(glasses));
    };
    #[cfg(feature = "nreal")]
    if let Ok(glasses) = NrealLight::new() {
        return Ok(Box::new(glasses));
    };
    #[cfg(feature = "mad_gaze")]
    if let Ok(glasses) = MadGazeGlow::new() {
        return Ok(Box::new(glasses));
    };
    Err(Error::NotFound)
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
