// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

//! Rokid Air AR glasses support. See [`RokidAir`]
//! It only uses [`rusb`] for communication.

use std::{io::Cursor, time::Duration};

use byteorder::{ReadBytesExt, LE};
use rusb::{request_type, DeviceHandle, GlobalContext};

use crate::{
    util::open_device_vid_pid_endpoint, ARGlasses, DisplayMode, GlassesEvent, Result, SensorData3D,
};

/// The main structure representing a connected Rokid Air glasses
pub struct RokidAir {
    device_handle: DeviceHandle<GlobalContext>,
    last_accelerometer: Option<SensorData3D>,
    last_gyroscope: Option<SensorData3D>,
    key_was_pressed: bool,
    proxy_sensor_was_far: bool,
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
                //eprintln!("{result:?}");
                let key_is_pressed = result[47] != 0;
                let send_key_event = key_is_pressed && !self.key_was_pressed;
                self.key_was_pressed = key_is_pressed;
                if send_key_event {
                    // We will lose the Proxy sensor "event" here,
                    // but we will get it in the next iteration soon.
                    return Ok(GlassesEvent::KeyPress(0));
                }
                let proxy_sensor_is_far = result[51] != 0;
                let send_proxy_event = proxy_sensor_is_far != self.proxy_sensor_was_far;
                self.proxy_sensor_was_far = proxy_sensor_is_far;
                if send_proxy_event {
                    return Ok(if proxy_sensor_is_far {
                        GlassesEvent::ProximityFar
                    } else {
                        GlassesEvent::ProximityNear
                    });
                }
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
                    1 => self.last_accelerometer = Some(sensor_data),
                    2 => self.last_gyroscope = Some(sensor_data),
                    // TODO: Magnetometer apparently gives an accuracy value too
                    3 => return Ok(GlassesEvent::Magnetometer(sensor_data)),
                    _ => (),
                }
                if let (Some(accelerometer), Some(gyroscope)) =
                    (self.last_accelerometer.clone(), self.last_gyroscope.clone())
                {
                    if accelerometer.timestamp == gyroscope.timestamp {
                        self.last_gyroscope = None;
                        self.last_accelerometer = None;
                        return Ok(GlassesEvent::AccGyro {
                            accelerometer,
                            gyroscope,
                        });
                    }
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
        let result = Self {
            device_handle: open_device_vid_pid_endpoint(0x04d2, 0x162f, INTERRUPT_IN_ENDPOINT)?,
            last_accelerometer: None,
            last_gyroscope: None,
            key_was_pressed: false,
            proxy_sensor_was_far: false,
        };
        Ok(result)
    }
}
