// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

//! Rokid Air AR glasses support. See [`RokidAir`]
//! It only uses [`rusb`] for communication.

use std::{io::Cursor, time::Duration};

use byteorder::{ReadBytesExt, LE};
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use rusb::{request_type, DeviceHandle, GlobalContext};

use crate::{
    util::get_interface_for_endpoint, ARGlasses, DisplayMode, Error, GlassesEvent, Result, Side,
};

/// The main structure representing a connected Rokid Air glasses
pub struct RokidAir {
    device_handle: DeviceHandle<GlobalContext>,
    last_accelerometer: Option<(Vector3<f32>, u64)>,
    last_gyroscope: Option<(Vector3<f32>, u64)>,
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
                let sensor_data = Vector3::new(x, y, z);
                match result[1] {
                    1 => self.last_accelerometer = Some((sensor_data, timestamp)),
                    2 => self.last_gyroscope = Some((sensor_data, timestamp)),
                    // TODO: Magnetometer apparently gives an accuracy value too
                    3 => {
                        return Ok(GlassesEvent::Magnetometer {
                            magnetometer: sensor_data,
                            timestamp,
                        })
                    }
                    _ => (),
                }
                if let (Some((accelerometer, acc_ts)), Some((gyroscope, gyro_ts))) =
                    (self.last_accelerometer, self.last_gyroscope)
                {
                    if acc_ts == gyro_ts {
                        self.last_gyroscope = None;
                        self.last_accelerometer = None;
                        return Ok(GlassesEvent::AccGyro {
                            accelerometer,
                            gyroscope,
                            timestamp: acc_ts,
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
        } else if result[1] == 1 {
            Ok(DisplayMode::Stereo)
        } else {
            Ok(DisplayMode::HighRefreshRate)
        }
    }

    fn set_display_mode(&mut self, display_mode: DisplayMode) -> Result<()> {
        let display_mode = match display_mode {
            DisplayMode::SameOnBoth => 0,
            DisplayMode::Stereo => 1,
            DisplayMode::HighRefreshRate => 3,
            _ => return Err(Error::Other("Display mode not supported")),
        };
        self.device_handle.write_control(
            request_type(
                rusb::Direction::Out,
                rusb::RequestType::Vendor,
                rusb::Recipient::Device,
            ),
            0x1,
            display_mode,
            0x1,
            &[0u8; 1],
            TIMEOUT,
        )?;
        Ok(())
    }

    fn display_fov(&self) -> f32 {
        // 21° is the advertised FOV
        // 20° is the (dynamically) measured one. It works better with normal PD settings
        20f32.to_radians()
    }

    fn imu_to_display_matrix(&self, side: Side, ipd: f32) -> Isometry3<f64> {
        let ipd = ipd as f64
            * match side {
                Side::Left => -0.5,
                Side::Right => 0.5,
            };
        Translation3::new(ipd, 0.0, 0.0) * UnitQuaternion::from_euler_angles(0.022, 0.0, 0.0)
    }

    fn name(&self) -> &'static str {
        "Rokid Air"
    }

    fn display_delay(&self) -> u64 {
        15000
    }
}

impl RokidAir {
    /// Vendor ID of the Rokid Air (Yes, it is 1234. Yes that's probably not very legit)
    pub const VID: u16 = 0x04d2;
    /// Product ID of the Rokid Air
    pub const PID: u16 = 0x162f;

    /// Connect to a specific glasses, based on the two USB fds
    /// Mainly made to work around android permission issues
    #[cfg(target_os = "android")]
    pub fn new(fd: isize) -> Result<Self> {
        use rusb::UsbContext;
        // Do not scan for devices in libusb_init()
        // This is needed on Android, where access to USB devices is limited
        unsafe { rusb::ffi::libusb_set_option(std::ptr::null_mut(), 2) };
        let mut device_handle = unsafe { GlobalContext::default().open_device_with_fd(fd as i32) }?;
        Self::new_common(device_handle)
    }

    /// Find a connected Rokid Air device and connect to it. (And claim the USB interface)
    /// Only one instance can be alive at a time
    #[cfg(not(target_os = "android"))]
    pub fn new() -> Result<Self> {
        use crate::util::get_device_vid_pid;

        Self::new_common(get_device_vid_pid(Self::VID, Self::PID)?.open()?)
    }

    fn new_common(mut device_handle: DeviceHandle<GlobalContext>) -> Result<Self> {
        device_handle.set_auto_detach_kernel_driver(true)?;

        device_handle.claim_interface(
            get_interface_for_endpoint(&device_handle.device(), INTERRUPT_IN_ENDPOINT).ok_or_else(
                || Error::Other("Could not find endpoint, wrong USB structure (probably)"),
            )?,
        )?;
        let result = Self {
            device_handle,
            last_accelerometer: None,
            last_gyroscope: None,
            key_was_pressed: false,
            proxy_sensor_was_far: false,
        };
        Ok(result)
    }
}
