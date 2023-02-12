// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

//! Rokid Air AR glasses support. See [`RokidAir`]
//! It only uses [`rusb`] for communication.

use std::{io::Cursor, time::Duration};

use byteorder::{ReadBytesExt, LE};
use rusb::{request_type, Device, DeviceHandle, DeviceList, GlobalContext};

use crate::{ARGlasses, DisplayMode, Error, GlassesEvent, MiscSensors, Result, SensorData3D};

/// The main structure representing a connected Rokid Air glasses
pub struct RokidAir {
    device_handle: DeviceHandle<GlobalContext>,
}

/* This is actually hardcoded in the SDK too, except for PID==0x162d, where it's 0x83 */
const INTERRUPT_IN_ENDPOINT: u8 = 0x82;

const TIMEOUT: Duration = Duration::from_millis(250);

impl ARGlasses for RokidAir {
    fn serial(&mut self) -> Result<String> {
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

    fn read_event(&mut self) -> Result<GlassesEvent> {
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

    fn get_display_mode(&mut self) -> Result<DisplayMode> {
        let mut result = [0; 0x40];
        self.device_handle.read_control(
            request_type(
                rusb::Direction::In,
                rusb::RequestType::Vendor,
                rusb::Recipient::Device,
            ),
            0x81,
            0x0,
            0x1,
            &mut result,
            TIMEOUT,
        )?;
        if result[1] == 0 {
            Ok(DisplayMode::SameOnBoth)
        } else {
            Ok(DisplayMode::Stereo)
        }
    }

    fn set_display_mode(&mut self, display_mode: DisplayMode) -> Result<()> {
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

impl RokidAir {
    /// Find a connected Rokid Air device and connect to it. (And claim the USB interface)
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
}
