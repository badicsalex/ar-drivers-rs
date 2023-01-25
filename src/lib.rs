// Copyright (C) 2023, Alex Badics
// This file is part of openreal
// Licensed under the MIT license. See LICENSE file in the project root for details.
#![warn(missing_docs)]
//! OpeNreal is a simplified driver for Nreal Air (and possibly other) AR glasses.
//! It supports getting basic sensor data and setting up the display.
//! It only uses [`rusb`] for communication.
//!
//! Example usage (in a thread, probably):
//! ```ignore
//! let glasses = Glasses::new().unwrap();
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
//! As opposed to Nreal's own API, this is all that you get, since this is what comes
//! out of the hardware. To get quaternions, you should probably use a lib that implements
//! Madgwicks algorithm or a proper EKF. One good choice is the `eskf` crate.

use std::{io::Cursor, time::Duration};

use byteorder::{ReadBytesExt, LE};
use rusb::{request_type, Device, DeviceHandle, DeviceList, GlobalContext};

/// The main structure representing a connected AR glasses
pub struct Glasses {
    device_handle: DeviceHandle<GlobalContext>,
}

/// Possible errors resulting from `openreal` API calls
#[derive(Debug, Clone)]
pub enum Error {
    /// An rusb error happened. See [`rusb::Error`] for specifics
    UsbError(rusb::Error),
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
    /// Keys pressed. On the Nreal Air, the only button is the brightness button,
    /// And it corresponds to the value `4`
    pub keys: u8,
    /// Proximity sensor report. `true` if the user is currently wearing the glasses
    pub proximity: bool,
}

/// Display mode used by [`Glasses::set_display_mode`]
pub enum DisplayMode {
    /// Picture should be same for both eyes (simple full HD mode)
    SameOnBoth = 0,
    /// Set display to 3840*1080, where the left half is the left eye, the right half
    /// is the right eye
    Stereo = 1,
}

/* This is actually hardcoded in the SDK too, except for PID==0x162d, where it's 0x83 */
const INTERRUPT_IN_ENDPOINT: u8 = 0x82;

const TIMEOUT: Duration = Duration::from_millis(250);

impl Glasses {
    /// Find a connected Nreal Air device and connect to it. (And claim the USB interface)
    /// Only one instance can be alive at a time
    pub fn new() -> Result<Self> {
        let device = Self::get_rusb_device()?;
        let interface_num = Self::get_interrupt_interface(&device)?;
        let mut device_handle = device.open()?;
        device_handle.set_auto_detach_kernel_driver(true)?;
        device_handle.claim_interface(interface_num)?;
        let result = Self { device_handle };
        Ok(result)
    }

    fn get_rusb_device() -> Result<Device<GlobalContext>> {
        for device in DeviceList::new()?.iter() {
            if let Ok(desc) = device.device_descriptor() {
                if desc.vendor_id() == 0x04d2 && desc.product_id() == 0x162f {
                    return Ok(device);
                }
            }
        }
        Err(Error::NotFound)
    }

    fn get_interrupt_interface(device: &Device<GlobalContext>) -> Result<u8> {
        let config_desc = device.config_descriptor(0)?;
        for interface in config_desc.interfaces() {
            for desc in interface.descriptors() {
                for endpoint in desc.endpoint_descriptors() {
                    if endpoint.address() == INTERRUPT_IN_ENDPOINT {
                        return Ok(interface.number());
                    }
                }
            }
        }
        Err(Error::Other(
            "Could not find endpoint, wrong USB structure (probably)",
        ))
    }

    /// Get the serial number of the glasses
    pub fn serial(&self) -> Result<String> {
        let mut result = [0u8; 0x40];
        self.device_handle.read_control(
            request_type(
                rusb::Direction::In,
                rusb::RequestType::Vendor,
                rusb::Recipient::Device,
            ),
            0x81,
            0x100,
            0,
            &mut result,
            TIMEOUT,
        )?;
        Ok(
            String::from_utf8(result.iter().copied().take_while(|c| *c != 0).collect())
                .map_err(|_| "Invalid serial string")?,
        )
    }

    /// Get a single sensor event. Blocks.
    pub fn read_event(&self) -> Result<GlassesEvent> {
        loop {
            let mut result = [0u8; 0x40];
            self.device_handle
                .read_interrupt(INTERRUPT_IN_ENDPOINT, &mut result, TIMEOUT)?;
            if result[0] == 2 {
                return Ok(GlassesEvent::Misc(MiscSensors {
                    keys: result[47],
                    proximity: result[51] == 0,
                }));
            }
            if result[0] == 4 {
                let mut cursor = Cursor::new(&result);
                cursor.set_position(9);
                let timestamp = cursor.read_u64::<LE>().unwrap();
                cursor.set_position(21);
                let x = cursor.read_f32::<LE>().unwrap();
                let y = cursor.read_f32::<LE>().unwrap();
                let z = cursor.read_f32::<LE>().unwrap();
                let sensor_data = SensorData3D { timestamp, x, y, z };
                match result[1] {
                    1 => return Ok(GlassesEvent::Accelerometer(sensor_data)),
                    2 => return Ok(GlassesEvent::Gyroscope(sensor_data)),
                    // TODO: Magnetometer apparently gives an accuracy value too
                    3 => return Ok(GlassesEvent::Magnetometer(sensor_data)),
                    _ => (),
                }
            }
        }
    }

    /// Set the display mode of the glasses. See [`DisplayMode`]
    pub fn set_display_mode(&self, display_mode: DisplayMode) -> Result<()> {
        self.device_handle.write_control(
            request_type(
                rusb::Direction::Out,
                rusb::RequestType::Vendor,
                rusb::Recipient::Device,
            ),
            0x1,
            display_mode as u16,
            0x1,
            &[0u8; 1],
            TIMEOUT,
        )?;
        Ok(())
    }
}

impl From<rusb::Error> for Error {
    fn from(e: rusb::Error) -> Self {
        Error::UsbError(e)
    }
}

impl From<&'static str> for Error {
    fn from(e: &'static str) -> Self {
        Error::Other(e)
    }
}
