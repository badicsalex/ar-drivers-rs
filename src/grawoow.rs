// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

//! Grawoow G530 (a.k.a. MetaVision M53) glasses support. See [`GrawoowG530`]
//! It only uses [`rusb`] for communication.

use std::time::{Duration, Instant};

use byteorder::{LittleEndian, ReadBytesExt};
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use rusb::{request_type, DeviceHandle, GlobalContext};
use tinyjson::JsonValue;

use crate::{
    util::get_interface_for_endpoint, ARGlasses, DisplayMode, Error, GlassesEvent, Result, Side,
};

/// The main structure representing a connected Grawoow G530 (a.k.a. MetaVision M53) glasses
pub struct GrawoowG530 {
    mcu_handle: DeviceHandle<GlobalContext>,
    ov580_handle: DeviceHandle<GlobalContext>,
    config_json: JsonValue,
    gyro_bias: Vector3<f32>,
    accelerometer_bias: Vector3<f32>,
    start: Instant,
}

const OV580_ENDPOINT: u8 = 0x89;

const OV580_TIMEOUT: Duration = Duration::from_millis(250);
const MCU_TIMEOUT: Duration = Duration::from_millis(1000);

impl ARGlasses for GrawoowG530 {
    fn serial(&mut self) -> Result<String> {
        Ok(String::from_utf8(self.command(0x8005, &[])?).map_err(|_| "Invalid serial string")?)
    }

    fn read_event(&mut self) -> Result<GlassesEvent> {
        let mut packet_data = [0u8; 0x80];
        self.ov580_handle
            .read_interrupt(OV580_ENDPOINT, &mut packet_data, OV580_TIMEOUT)?;
        self.parse_imu_packet(&packet_data)
    }

    fn get_display_mode(&mut self) -> Result<DisplayMode> {
        let result = self.command(0x8007, &[])?;
        if result.first() == Some(&1) {
            Ok(DisplayMode::Stereo)
        } else {
            Ok(DisplayMode::SameOnBoth)
        }
    }

    fn set_display_mode(&mut self, display_mode: DisplayMode) -> Result<()> {
        let display_mode = match display_mode {
            DisplayMode::SameOnBoth => 0,
            DisplayMode::Stereo => 1,
            _ => return Err(Error::Other("Display mode not supported")),
        };
        self.command(0x8008, &[display_mode])?;
        Ok(())
    }

    fn display_fov(&self) -> f32 {
        // Measurement result
        22f32.to_radians()
    }

    fn imu_to_display_matrix(&self, side: Side, ipd: f32) -> Isometry3<f64> {
        // TODO: use calibration data if possible
        let side_multiplier = match side {
            Side::Left => -0.5,
            Side::Right => 0.5,
        };
        Translation3::new(ipd as f64 * side_multiplier, 0.0, 0.0)
            * UnitQuaternion::from_euler_angles(
                Self::DISPLAY_TILT,
                Self::DISPLAY_DIVERGENCE * side_multiplier,
                0.0,
            )
    }

    fn name(&self) -> &'static str {
        "Grawoow G530"
    }

    fn display_delay(&self) -> u64 {
        15000
    }
}

impl GrawoowG530 {
    /// Vendor ID of the MCU on the G530
    pub const MCU_VID: u16 = 0x1ff7;
    /// Product ID of the MCU on the G530
    pub const MCU_PID: u16 = 0x0ff4;

    /// Vendor ID of the OV580 on the G530
    pub const OV580_VID: u16 = 0x05a9;
    /// Product ID of the OV580 on the G530
    pub const OV580_PID: u16 = 0x0f87;

    const DISPLAY_TILT: f64 = -0.11;
    const DISPLAY_DIVERGENCE: f64 = 0.02;

    /// Connect to a specific glasses, based on the two USB fds
    /// Mainly made to work around android permission issues
    #[cfg(target_os = "android")]
    pub fn new(mcu_fd: isize, ov580_fd: isize) -> Result<Self> {
        use rusb::UsbContext;
        // Do not scan for devices in libusb_init()
        // This is needed on Android, where access to USB devices is limited
        unsafe { rusb::ffi::libusb_set_option(std::ptr::null_mut(), 2) };
        Self::new_common(
            unsafe { GlobalContext::default().open_device_with_fd(mcu_fd as i32) }?,
            unsafe { GlobalContext::default().open_device_with_fd(ov580_fd as i32) }?,
        )
    }

    /// Find a connected device and connect to it. (And claim the USB interface)
    /// Only one instance can be alive at a time
    #[cfg(not(target_os = "android"))]
    pub fn new() -> Result<Self> {
        use crate::util::get_device_vid_pid;

        Self::new_common(
            get_device_vid_pid(Self::MCU_VID, Self::MCU_PID)?.open()?,
            get_device_vid_pid(Self::OV580_VID, Self::OV580_PID)?.open()?,
        )
    }

    fn new_common(
        mut mcu_handle: DeviceHandle<GlobalContext>,
        mut ov580_handle: DeviceHandle<GlobalContext>,
    ) -> Result<Self> {
        mcu_handle.set_auto_detach_kernel_driver(true)?;
        ov580_handle.set_auto_detach_kernel_driver(true)?;

        mcu_handle.claim_interface(0)?;
        ov580_handle.claim_interface(
            get_interface_for_endpoint(&ov580_handle.device(), OV580_ENDPOINT).ok_or_else(
                || Error::Other("Could not find endpoint, wrong USB structure (probably)"),
            )?,
        )?;
        let mut result = Self {
            mcu_handle,
            ov580_handle,
            config_json: tinyjson::JsonValue::Null,
            gyro_bias: Default::default(),
            accelerometer_bias: Default::default(),
            start: Instant::now(),
        };
        result.read_calibration()?;
        Ok(result)
    }

    fn read_calibration(&mut self) -> Result<()> {
        let mut calib_string = Vec::new();
        for _ in 0..100 {
            let part = self.command(
                0x800a,
                &[
                    0,
                    0,
                    0,
                    (calib_string.len() >> 8) as u8,
                    calib_string.len() as u8,
                ],
            )?;
            if part.len() <= 6 {
                break;
            }
            calib_string.extend_from_slice(&part[6..]);
        }

        self.config_json = String::from_utf8(calib_string)
            .map_err(|_| Error::Other("Invalid glasses config format (no start token)"))?
            .parse()
            .map_err(|_| Error::Other("Invalid glasses config format (JSON parse error)"))?;

        let json = &self.config_json["imu"][0]["RM_acc"];
        self.accelerometer_bias = Vector3::new(
            *json[9].get::<f64>().unwrap() as f32,
            *json[10].get::<f64>().unwrap() as f32,
            *json[11].get::<f64>().unwrap() as f32,
        );
        let json = &self.config_json["imu"][0]["RM_gyro"];
        self.gyro_bias = Vector3::new(
            *json[9].get::<f64>().unwrap() as f32,
            *json[10].get::<f64>().unwrap() as f32,
            *json[11].get::<f64>().unwrap() as f32,
        );
        Ok(())
    }

    fn command(&self, cmd_id: u16, additional_data: &[u8]) -> Result<Vec<u8>> {
        self.send_command_request(cmd_id, additional_data)?;
        self.recv_command_result(cmd_id)
    }

    fn send_command_request(&self, cmd_id: u16, additional_data: &[u8]) -> Result<()> {
        let mut control_data = vec![
            0xaa,
            0xbb,
            (cmd_id >> 8) as u8,
            cmd_id as u8,
            0,
            additional_data.len() as u8,
        ];
        control_data.extend_from_slice(additional_data);
        let checksum: u32 = control_data[2..].iter().map(|x| *x as u32).sum();
        control_data.push(checksum as u8);
        let control_data = &control_data[..6 + additional_data.len() + 1];

        self.mcu_handle.write_control(
            request_type(
                rusb::Direction::Out,
                rusb::RequestType::Class,
                rusb::Recipient::Interface,
            ),
            9,
            0x201,
            0,
            control_data,
            MCU_TIMEOUT,
        )?;
        Ok(())
    }
    fn recv_command_result(&self, cmd_id: u16) -> Result<Vec<u8>> {
        let mut result = [0; 0x100];
        self.mcu_handle.read_control(
            request_type(
                rusb::Direction::In,
                rusb::RequestType::Class,
                rusb::Recipient::Interface,
            ),
            1,
            0x102,
            0,
            &mut result,
            MCU_TIMEOUT,
        )?;
        if result[0] != 0xaa
            || result[1] != 0xbb
            || result[2] != (cmd_id >> 8) as u8
            || result[3] != (cmd_id & 0xff) as u8
            || result[4] != 0
        // TODO: check checksum
        {
            return Err(Error::Other("Protocol error"));
        }
        let len = result[5] as usize;
        Ok(result[6..(6 + len)].into())
    }

    fn parse_imu_packet(&self, data: &[u8]) -> Result<GlassesEvent> {
        const GYRO_MUL: f32 = std::f32::consts::PI / 180.0 / 16.4;
        const ACC_MUL: f32 = 9.81 / 16384.0;
        let mut reader = std::io::Cursor::new(&data);
        reader.set_position(0x3c);
        let gyro_x = reader.read_i32::<LittleEndian>()? as f32;
        let gyro_y = reader.read_i32::<LittleEndian>()? as f32;
        let gyro_z = reader.read_i32::<LittleEndian>()? as f32;
        let gyroscope = Vector3::new(
            -(gyro_y * GYRO_MUL - self.gyro_bias.y),
            -(gyro_z * GYRO_MUL - self.gyro_bias.z),
            gyro_x * GYRO_MUL - self.gyro_bias.x,
        );

        reader.set_position(0x58);
        let acc_x = reader.read_i32::<LittleEndian>()? as f32;
        let acc_y = reader.read_i32::<LittleEndian>()? as f32;
        let acc_z = reader.read_i32::<LittleEndian>()? as f32;
        let accelerometer = Vector3::new(
            -(acc_y * ACC_MUL - self.accelerometer_bias.y),
            -(acc_z * ACC_MUL - self.accelerometer_bias.z),
            acc_x * ACC_MUL - self.accelerometer_bias.x,
        );
        Ok(GlassesEvent::AccGyro {
            accelerometer,
            gyroscope,
            timestamp: self.start.elapsed().as_micros() as u64,
        })
    }
}
