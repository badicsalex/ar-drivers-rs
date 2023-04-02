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

use mad_gaze::MadGazeGlow;
use nreal::NrealLight;
use rokid::RokidAir;

pub mod mad_gaze;
pub mod rokid;
pub mod nreal;

/// Possible errors resulting from `ar-drivers` API calls
#[derive(Debug)]
pub enum Error {
    /// A standard IO error happened. See [`std::io::Error`] for specifics
    IoError(std::io::Error),
    /// An rusb error happened. See [`rusb::Error`] for specifics
    UsbError(rusb::Error),
    /// A serialport error happened. See [`serialport::Error`] for specifics
    SerialPortError(serialport::Error),
    /// No glasses were found.
    NotFound,
    /// Other fatal error, usually a problem with the library itself, or
    /// a device support issue. File a bug if you encounter this.
    Other(&'static str),
}

type Result<T> = std::result::Result<T, Error>;

/// AR glasses sensor event, got from [`Glasses::read_event`]
#[derive(Debug, Clone)]
pub enum GlassesEvent {
    /// Accelerometer data in m^2/s
    Accelerometer(SensorData3D),
    /// Gyroscope data. Right handed rotation in rad/sec,
    /// e.g. turning left is positive y axis.
    Gyroscope(SensorData3D),
    /// Magnetometer data. Vector points to magnetic north (mostly)
    Magnetometer(SensorData3D),
    /// Other sensor data
    Misc(MiscSensors),
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

/// Container for other sensor data
#[derive(Debug, Clone)]
pub struct MiscSensors {
    /// Keys pressed. On the Rokid Air, the only button is the brightness button,
    /// And it corresponds to the value `4`
    pub keys: u8,
    /// Proximity sensor report. `true` if the user is currently wearing the glasses
    pub proximity: bool,
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
pub trait ARGlasses {
    /// Get the serial number of the glasses
    fn serial(&mut self) -> Result<String>;
    /// Get a single sensor event. Blocks.
    fn read_event(&mut self) -> Result<GlassesEvent>;
    /// Get the display mode of the glasses. See [`DisplayMode`]
    fn get_display_mode(&mut self) -> Result<DisplayMode>;
    /// Set the display mode of the glasses. See [`DisplayMode`]
    fn set_display_mode(&mut self, display_mode: DisplayMode) -> Result<()>;
}

/// Convenience function to detect and connect to any of the supported glasses
pub fn any_glasses() -> Result<Box<dyn ARGlasses>> {
    if let Ok(glasses) = RokidAir::new() {
        return Ok(Box::new(glasses));
    };
    if let Ok(glasses) = NrealLight::new() {
        return Ok(Box::new(glasses));
    };
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

impl From<rusb::Error> for Error {
    fn from(e: rusb::Error) -> Self {
        Error::UsbError(e)
    }
}

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
