// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

//! Rokid Air AR glasses support. See [`RokidAir`]
//! It only uses [`rusb`] for communication.

use std::{collections::VecDeque, time::Duration};

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
    previous_key_states: u8,
    proxy_sensor_was_far: bool,
    pending_events: VecDeque<GlassesEvent>,
    model: RokidModel,
}

enum RokidModel {
    Air,
    Max,
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
        while self.pending_events.is_empty() {
            let mut packet_data = [0u8; 0x40];
            self.device_handle
                .read_interrupt(INTERRUPT_IN_ENDPOINT, &mut packet_data, TIMEOUT)?;
            match packet_data[0] {
                2 => {
                    let packet: &MiscPacket = bytemuck::cast_ref(&packet_data);
                    self.handle_key_press(packet.keys_pressed);
                    self.handle_proxy_sensor(packet.proxy_sensor);
                }
                4 => {
                    let packet: &SensorPacket = bytemuck::cast_ref(&packet_data);
                    let sensor_data =
                        Vector3::from_data(nalgebra::ArrayStorage([packet.vector; 1]));
                    match packet.sensor_type {
                        1 => self.last_accelerometer = Some((sensor_data, packet.timestamp)),
                        2 => self.last_gyroscope = Some((sensor_data, packet.timestamp)),
                        // TODO: Magnetometer apparently gives an accuracy value too
                        3 => self.pending_events.push_back(GlassesEvent::Magnetometer {
                            magnetometer: sensor_data,
                            timestamp: packet.timestamp,
                        }),
                        _ => (),
                    }
                    if let (Some((accelerometer, acc_ts)), Some((gyroscope, gyro_ts))) =
                        (self.last_accelerometer, self.last_gyroscope)
                    {
                        if acc_ts == gyro_ts {
                            self.last_gyroscope = None;
                            self.last_accelerometer = None;
                            self.pending_events.push_back(GlassesEvent::AccGyro {
                                accelerometer,
                                gyroscope,
                                timestamp: acc_ts,
                            });
                        }
                    }
                }
                17 => {
                    let packet: &CombinedPacket = bytemuck::cast_ref(&packet_data);
                    let timestamp = packet.timestamp / 1000;
                    self.pending_events.push_back(GlassesEvent::AccGyro {
                        accelerometer: Vector3::from_data(nalgebra::ArrayStorage(
                            [packet.accelerometer; 1],
                        )),
                        gyroscope: Vector3::from_data(nalgebra::ArrayStorage(
                            [packet.gyroscope; 1],
                        )),
                        timestamp,
                    });
                    self.pending_events.push_back(GlassesEvent::Magnetometer {
                        magnetometer: Vector3::from_data(nalgebra::ArrayStorage(
                            [packet.magnetometer; 1],
                        )),
                        timestamp,
                    });
                    // NOTE: was always zero on my Max
                    self.handle_key_press(packet.keys_pressed);
                    self.handle_proxy_sensor(packet.proxy_sensor);
                }
                _ => {}
            }
        }
        Ok(self.pending_events.pop_front().unwrap())
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
        match result[1] {
            0 => Ok(DisplayMode::SameOnBoth),
            1 => Ok(DisplayMode::Stereo),
            2 => Ok(DisplayMode::HalfSBS),
            4 => Ok(DisplayMode::HighRefreshRateSBS),
            _ => Ok(DisplayMode::HighRefreshRate),
        }
    }

    fn set_display_mode(&mut self, display_mode: DisplayMode) -> Result<()> {
        let display_mode = match display_mode {
            DisplayMode::SameOnBoth => 0,
            DisplayMode::Stereo => 1,
            DisplayMode::HighRefreshRate => 3,
            DisplayMode::HighRefreshRateSBS => 4,
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
        match self.model {
            RokidModel::Air => {
                // 21° is the advertised FOV
                // 20° is the (dynamically) measured one. It works better with normal PD settings
                20f32.to_radians()
            }
            RokidModel::Max => {
                // Measured
                23f32.to_radians()
            }
        }
    }

    fn imu_to_display_matrix(&self, side: Side, ipd: f32) -> Isometry3<f64> {
        let tilt = match self.model {
            RokidModel::Air => 0.022,
            RokidModel::Max => 0.07,
        };
        let ipd = ipd as f64
            * match side {
                Side::Left => -0.5,
                Side::Right => 0.5,
            };
        Translation3::new(ipd, 0.0, 0.0) * UnitQuaternion::from_euler_angles(tilt, 0.0, 0.0)
    }

    fn name(&self) -> &'static str {
        match self.model {
            RokidModel::Air => "Rokid Air",
            RokidModel::Max => "Rokid Max",
        }
    }

    fn display_delay(&self) -> u64 {
        match self.model {
            RokidModel::Air => 15000,
            RokidModel::Max => 13000,
        }
    }
}

#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
struct MiscPacket {
    packet_type: u8,
    seq: u32,
    _unknown_0: [u8; 42],
    keys_pressed: u8,
    _unknown_1: [u8; 3],
    proxy_sensor: u8,
    _unknown_2: [u8; 12],
}

unsafe impl bytemuck::Zeroable for MiscPacket {}
unsafe impl bytemuck::Pod for MiscPacket {}

#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
struct SensorPacket {
    packet_type: u8,
    sensor_type: u8,
    seq: u32,
    _unknown_0: [u8; 3],
    timestamp: u64,
    _unknown_1: [u8; 4],
    vector: [f32; 3],
    _unknown_2: [u8; 31],
}

unsafe impl bytemuck::Zeroable for SensorPacket {}
unsafe impl bytemuck::Pod for SensorPacket {}

#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
struct CombinedPacket {
    packet_type: u8,
    timestamp: u64,
    accelerometer: [f32; 3],
    gyroscope: [f32; 3],
    magnetometer: [f32; 3],
    keys_pressed: u8,
    proxy_sensor: u8,
    _unknown_0: u8,
    vsync_timestamp: u64,
    _unknown_1: [u8; 3],
    display_brightness: u8,
    volume: u8,
    _unknown_2: [u8; 3],
}

unsafe impl bytemuck::Zeroable for CombinedPacket {}
unsafe impl bytemuck::Pod for CombinedPacket {}

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
        let device_handle = unsafe { GlobalContext::default().open_device_with_fd(fd as i32) }?;
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
        let product_string = device_handle
            .read_product_string_ascii(&device_handle.device().device_descriptor()?)?;
        let result = Self {
            device_handle,
            last_accelerometer: None,
            last_gyroscope: None,
            previous_key_states: 0,
            proxy_sensor_was_far: false,
            model: if product_string.contains("Max") {
                RokidModel::Max
            } else {
                RokidModel::Air
            },
            pending_events: Default::default(),
        };
        Ok(result)
    }

    fn handle_key_press(&mut self, keys_pressed: u8) {
        let new_presses = keys_pressed & !self.previous_key_states;
        for bit in 0..8 {
            if new_presses & (1 << bit) != 0 {
                self.pending_events.push_back(GlassesEvent::KeyPress(bit))
            }
        }
        self.previous_key_states = keys_pressed;
    }

    fn handle_proxy_sensor(&mut self, value: u8) {
        let proxy_sensor_is_far = value != 0;
        let send_proxy_event = proxy_sensor_is_far != self.proxy_sensor_was_far;
        self.proxy_sensor_was_far = proxy_sensor_is_far;
        if send_proxy_event {
            self.pending_events.push_back(if proxy_sensor_is_far {
                GlassesEvent::ProximityFar
            } else {
                GlassesEvent::ProximityNear
            });
        }
    }
}
