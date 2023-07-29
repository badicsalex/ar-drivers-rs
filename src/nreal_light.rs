// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

//! Nreal Light AR glasses support. See [`NrealLight`]
//! It only uses [`hidapi`] for communication.
//!
//! **Important note**: The NReal Light requires constant heartbeats in 3D SBS mode,
//! or else it switches the screen off. This heartbeat is sent periodically when
//! [`NrealLight::read_event`] is called, so be sure to constantly call that function (at least once
//! every half a second or so)

use std::{collections::VecDeque, io::Write};

use byteorder::{LittleEndian, ReadBytesExt};
use hidapi::{HidApi, HidDevice};
#[cfg(feature = "nalgebra")]
use nalgebra::{Isometry3, Matrix3, Quaternion, UnitQuaternion, Vector2, Vector4};
use tinyjson::JsonValue;

#[cfg(feature = "nalgebra")]
use crate::CameraDescriptor;
use crate::{
    util::{crc32_adler, Vector3},
    ARGlasses, DisplayMode, Error, GlassesEvent, Result, SensorData3D,
};

/// The main structure representing a connected Nreal Light glasses
pub struct NrealLight {
    device: HidDevice,
    pending_packets: VecDeque<Packet>,
    last_heartbeat: std::time::Instant,
    ov580: Ov580,
}

const COMMAND_TIMEOUT: i32 = 250;
const OV_580_TIMEOUT: i32 = 250;

impl ARGlasses for NrealLight {
    fn serial(&mut self) -> Result<String> {
        let result = self.run_command(Packet {
            category: b'3',
            cmd_id: b'C',
            ..Default::default()
        })?;
        String::from_utf8(result).map_err(|_| Error::Other("Serial number was not utf-8"))
    }

    fn read_event(&mut self) -> Result<GlassesEvent> {
        self.send_heartbeat_if_needed()?;
        if let Some(event) = self.read_mcu_packet()? {
            Ok(event)
        } else {
            self.ov580.read_packet()
        }
    }

    fn get_display_mode(&mut self) -> Result<DisplayMode> {
        let result = self.run_command(Packet {
            category: b'3',
            cmd_id: b'3',
            ..Default::default()
        })?;
        match result.first() {
            // "1&2D_1080"
            Some(b'1') => Ok(DisplayMode::SameOnBoth),
            // "2&3D_540"
            Some(b'2') => Ok(DisplayMode::HalfSBS),
            // "3&3D_1080"
            Some(b'3') => Ok(DisplayMode::Stereo),
            // "4&3D_1080#72"
            Some(b'4') => Ok(DisplayMode::HighRefreshRate),
            _ => Err(Error::Other("Unknown display mode")),
        }
    }

    fn set_display_mode(&mut self, display_mode: DisplayMode) -> Result<()> {
        let display_mode_byte = match display_mode {
            DisplayMode::SameOnBoth => b'1',
            DisplayMode::HalfSBS => b'2',
            DisplayMode::Stereo => b'3',
            DisplayMode::HighRefreshRate => b'4',
        };
        let result = self.run_command(Packet {
            category: b'1',
            cmd_id: b'3',
            data: vec![display_mode_byte],
        })?;

        if result.first() == Some(&display_mode_byte) {
            Ok(())
        } else {
            Err(Error::Other("Display mode setting unsuccessful"))
        }
    }

    fn display_fov(&self) -> f32 {
        // 24.0° is the advertised FOV
        // According to measurements, it is even a bit better
        25.0f32.to_radians()
    }

    fn display_tilt(&self) -> f32 {
        -0.265
    }

    fn name(&self) -> &'static str {
        "Nreal Light"
    }

    #[cfg(feature = "nalgebra")]
    fn cameras(&self) -> Result<Vec<crate::CameraDescriptor>> {
        let rgb = self.get_basic_camera_descriptor("rgb", "RGB_camera", "device_1")?;
        let slam_left =
            self.get_basic_camera_descriptor("Nreal Light SLAM left", "SLAM_camera", "device_1")?;
        let mut slam_right =
            self.get_basic_camera_descriptor("Nreal Light SLAM right", "SLAM_camera", "device_2")?;
        slam_right.stereo_rotation = self
            .get_config_float_array(&["SLAM_camera", "leftcam_q_rightcam"])
            .map(Vector4::from_data)
            .map(Quaternion::from_vector)
            .map(UnitQuaternion::from_quaternion)?
            .conjugate();

        Ok(vec![rgb, slam_left, slam_right])
    }
}

impl NrealLight {
    /// Vendor ID of the NReal Light's MCU
    pub const MCU_VID: u16 = 0x0486;
    /// Product ID of the NReal Light's MCU
    pub const MCU_PID: u16 = 0x573c;

    /// Vendor ID of the NReal Light's OV580 camera (and IMU)
    pub const OV580_VID: u16 = 0x05a9;
    /// Product ID of the NReal Light's OV580 camera (and IMU)
    pub const OV580_PID: u16 = 0x0680;

    /// Connect to a specific glasses, based on the two USB fds
    /// Mainly made to work around android permission issues
    #[cfg(target_os = "android")]
    pub fn new(mcu_fd: isize, ov580_fd: isize) -> Result<Self> {
        Self::new_common(
            HidApi::new_without_enumerate()?.wrap_sys_device(mcu_fd, -1)?,
            Ov580::new(ov580_fd)?,
        )
    }

    /// Find a connected Nreal Light device and connect to it. (And claim the USB interface)
    /// Only one instance can be alive at a time
    #[cfg(not(target_os = "android"))]
    pub fn new() -> Result<Self> {
        Self::new_common(
            HidApi::new()?.open(Self::MCU_VID, Self::MCU_PID)?,
            Ov580::new()?,
        )
    }
    fn new_common(device: HidDevice, ov580: Ov580) -> Result<Self> {
        let mut result = Self {
            device,
            pending_packets: Default::default(),
            last_heartbeat: std::time::Instant::now(),
            ov580,
        };
        // Send a "Yes, I am a working SDK" command
        // This is needed for SBS 3D display to work.
        result.run_command(Packet {
            category: b'@',
            cmd_id: b'3',
            data: vec![b'1'],
        })?;
        // Enable the Ambient Light event
        result.run_command(Packet {
            category: b'1',
            cmd_id: b'L',
            data: vec![b'1'],
        })?;
        // Enable VSync event
        result.run_command(Packet {
            category: b'1',
            cmd_id: b'N',
            data: vec![b'1'],
        })?;
        Ok(result)
    }

    /// Returns the calibration data stored on the Glasses. No transformation
    /// is done on the data, except for Json Parsing.
    pub fn get_config_json(&self) -> &JsonValue {
        &self.ov580.config_json
    }

    fn read_mcu_packet(&mut self) -> Result<Option<GlassesEvent>> {
        let packet = if let Some(packet) = self.pending_packets.pop_front() {
            packet
        } else if let Some(packet) = self.read_packet(0)? {
            packet
        } else {
            return Ok(None);
        };
        Ok(match packet {
            Packet {
                category: b'5',
                cmd_id: b'K',
                data,
            } if data == b"UP" => Some(GlassesEvent::KeyPress(0)),
            Packet {
                category: b'5',
                cmd_id: b'K',
                data,
            } if data == b"DN" => Some(GlassesEvent::KeyPress(1)),
            Packet {
                category: b'5',
                cmd_id: b'P',
                data,
            } if data == b"near" => Some(GlassesEvent::ProximityNear),
            Packet {
                category: b'5',
                cmd_id: b'P',
                data,
            } if data == b"away" => Some(GlassesEvent::ProximityFar),
            Packet {
                category: b'5',
                cmd_id: b'L',
                data,
            } => Some(GlassesEvent::AmbientLight(
                u16::from_str_radix(
                    &String::from_utf8(data)
                        .map_err(|_| Error::Other("Invalid utf-8 in ambient light msg"))?,
                    16,
                )
                .map_err(|_| Error::Other("Invalid number in ambient light msg"))?,
            )),
            Packet {
                category: b'5',
                cmd_id: b'S',
                ..
            } => Some(GlassesEvent::VSync),
            // NOTE: maybe we should retry right here instead of basically reporting timeout,
            //       but we will be called again soon enough.
            _ => None,
        })
    }

    fn read_packet(&mut self, timeout: i32) -> Result<Option<Packet>> {
        let mut result = [0u8; 0x40];
        let packet_size = self.device.read_timeout(&mut result, timeout)?;
        if packet_size == 0 {
            Ok(None)
        } else {
            Ok(Some(
                Packet::deserialize(&result).ok_or(Error::Other("Malformed packet received"))?,
            ))
        }
    }

    fn send_heartbeat_if_needed(&mut self) -> Result<()> {
        let now = std::time::Instant::now();
        if now.duration_since(self.last_heartbeat) > std::time::Duration::from_millis(250) {
            // Heartbeat packet
            // Not sent as "run_command" as sometimes the Glasses don't bother to
            // answer. E.g. when one of the buttons is pressed while it is running.
            self.device.write(
                &Packet {
                    category: b'@',
                    cmd_id: b'K',
                    ..Default::default()
                }
                .serialize()
                .ok_or(Error::Other("Packet serialization failed"))?,
            )?;
            self.last_heartbeat = now;
        }
        Ok(())
    }

    fn run_command(&mut self, command: Packet) -> Result<Vec<u8>> {
        self.device.write(
            &command
                .serialize()
                .ok_or(Error::Other("Packet serialization failed"))?,
        )?;

        for _ in 0..64 {
            let packet = self
                .read_packet(COMMAND_TIMEOUT)?
                .ok_or(Error::PacketTimeout)?;
            if packet.category == command.category + 1 && packet.cmd_id == command.cmd_id {
                return Ok(packet.data);
            }
            self.pending_packets.push_back(packet);
        }

        Err(Error::Other("Received too many unrelated packets"))
    }

    #[cfg(feature = "nalgebra")]
    fn get_config_float_array<const N: usize>(
        &self,
        keys: &[&str],
    ) -> Result<nalgebra::ArrayStorage<f64, N, 1>> {
        use std::collections::HashMap;

        let mut result = [0.0; N];
        let mut json_val = self.get_config_json();
        for key in keys {
            json_val = json_val
                .get::<HashMap<String, JsonValue>>()
                .ok_or(Error::Other("Json value is not an object"))?
                .get(*key)
                .ok_or(Error::Other("Json key not found"))?;
        }
        let json_val = json_val
            .get::<Vec<JsonValue>>()
            .ok_or(Error::Other("Json value not an array"))?;
        if json_val.len() != N {
            return Err(Error::Other("Json array is the wrong length"));
        }
        for i in 0..N {
            result[i] = *json_val[i]
                .get::<f64>()
                .ok_or(Error::Other("Json value is not a float"))?;
        }
        Ok(nalgebra::ArrayStorage([result]))
    }

    #[cfg(feature = "nalgebra")]
    fn get_basic_camera_descriptor(
        &self,
        name: &'static str,
        k1: &str,
        k2: &str,
    ) -> Result<CameraDescriptor> {
        let imu_p_cam = self
            .get_config_float_array(&[k1, k2, "imu_p_cam"])
            .map(Vector3::from_data);
        let imu_q_cam = self
            .get_config_float_array(&[k1, k2, "imu_q_cam"])
            .map(Vector4::from_data)
            .map(Quaternion::from_vector)
            .map(UnitQuaternion::from_quaternion);

        Ok(CameraDescriptor {
            name,
            resolution: Vector2::from_data(self.get_config_float_array(&[k1, k2, "resolution"])?),
            intrinsic_matrix: {
                let cc = Vector2::from_data(self.get_config_float_array(&[k1, k2, "cc"])?);
                let fc = Vector2::from_data(self.get_config_float_array(&[k1, k2, "fc"])?);
                Matrix3::new(fc.x, 0.0, cc.x, 0.0, fc.y, cc.y, 0.0, 0.0, 1.0)
            },
            distortion: self.get_config_float_array(&[k1, k2, "kc"])?.0[0],
            stereo_rotation: Default::default(),
            imu_to_camera: if let (Ok(p), Ok(q)) = (imu_p_cam, imu_q_cam) {
                Isometry3::from_parts(p.into(), q)
            } else {
                Default::default()
            },
        })
    }
}

struct Ov580 {
    device: HidDevice,
    config_json: JsonValue,
    gyro_bias: Vector3<f32>,
    accelerometer_bias: Vector3<f32>,
}

impl Ov580 {
    #[cfg(target_os = "android")]
    pub fn new(fd: isize) -> Result<Self> {
        Self::new_device(HidApi::new_without_enumerate()?.wrap_sys_device(fd, -1)?)
    }

    #[cfg(not(target_os = "android"))]
    pub fn new() -> Result<Self> {
        Self::new_device(HidApi::new()?.open(NrealLight::OV580_VID, NrealLight::OV580_PID)?)
    }
    fn new_device(device: HidDevice) -> Result<Self> {
        let mut result = Self {
            device,
            config_json: JsonValue::Null,
            gyro_bias: Default::default(),
            accelerometer_bias: Default::default(),
        };
        // Turn off IMU stream while reading config
        result.command(0x19, 0x0)?;
        result.read_config()?;
        result.parse_config()?;
        // Turn IMU stream back on
        result.command(0x19, 0x1)?;

        Ok(result)
    }

    fn read_config(&mut self) -> Result<()> {
        // Start reading config
        self.command(0x14, 0x0)?;
        let mut config = Vec::new();
        loop {
            let config_part = self.command(0x15, 0x0)?;
            if config_part[0] != 2 || config_part[1] != 1 {
                break;
            }
            config.extend_from_slice(&config_part[3..(3 + config_part[2] as usize)]);
        }
        for i in 0x28..config.len() - 4 {
            if config[i..i + 3] == [b'\n', b'\n', b'{'] {
                let config_as_str = String::from_utf8(config[i + 2..].into())
                    .map_err(|_| Error::Other("Invalid glasses config format (no start token)"))?;
                self.config_json = config_as_str
                    .split_once("\n\n")
                    .ok_or(Error::Other("Invalid glasses config format (no end token)"))?
                    .0
                    .parse()
                    .map_err(|_| {
                        Error::Other("Invalid glasses config format (JSON parse error)")
                    })?;
            }
        }
        Ok(())
    }

    fn parse_config(&mut self) -> Result<()> {
        // XXX: This will panic if config is not in expected format.
        //      should probably return Err() instead.
        let cfg = &self.config_json["IMU"]["device_1"];
        self.accelerometer_bias = Self::parse_vector(&cfg["accel_bias"]);
        self.gyro_bias = Self::parse_vector(&cfg["gyro_bias"]);
        Ok(())
    }

    fn parse_vector(json: &JsonValue) -> Vector3<f32> {
        Vector3::new(
            *json[0].get::<f64>().unwrap() as f32,
            *json[1].get::<f64>().unwrap() as f32,
            *json[2].get::<f64>().unwrap() as f32,
        )
    }

    fn command(&self, cmd: u8, subcmd: u8) -> Result<Vec<u8>> {
        self.device.write(&[2, cmd, subcmd, 0, 0, 0, 0])?;
        for _ in 0..64 {
            let mut result = [0u8; 0x80];
            let result_size = self.device.read_timeout(&mut result, OV_580_TIMEOUT)?;
            if result_size == 0 {
                return Err(Error::PacketTimeout);
            }
            if result[0] == 2 {
                return Ok(result.into());
            }
        }
        Err(Error::Other("Couldn't get acknowledgement to command"))
    }

    pub fn read_packet(&mut self) -> Result<GlassesEvent> {
        loop {
            let mut packet_data = [0u8; 0x80];
            let data_size = self.device.read_timeout(&mut packet_data, OV_580_TIMEOUT)?;
            if data_size == 0 {
                return Err(Error::PacketTimeout);
            }

            if packet_data[0] == 1 {
                return self.parse_report(&packet_data);
            };
            // Else try again
        }
    }

    fn parse_report(&mut self, packet_data: &[u8]) -> Result<GlassesEvent> {
        // TODO: This skips over a 2 byte temperature field that may be useful.
        let mut reader = std::io::Cursor::new(&packet_data[44..]);

        let gyro_timestamp = reader.read_u64::<LittleEndian>()? / 1000;
        let gyro_mul = reader.read_u32::<LittleEndian>()? as f32;
        let gyro_div = reader.read_u32::<LittleEndian>()? as f32;
        let gyro_x = reader.read_i32::<LittleEndian>()? as f32;
        let gyro_y = reader.read_i32::<LittleEndian>()? as f32;
        let gyro_z = reader.read_i32::<LittleEndian>()? as f32;
        let gyroscope = SensorData3D {
            timestamp: gyro_timestamp,
            x: (gyro_x * gyro_mul / gyro_div).to_radians() - self.gyro_bias.x,
            y: -(gyro_y * gyro_mul / gyro_div).to_radians() + self.gyro_bias.y,
            z: -(gyro_z * gyro_mul / gyro_div).to_radians() + self.gyro_bias.z,
        };

        let acc_timestamp = reader.read_u64::<LittleEndian>()? / 1000;
        let acc_mul = reader.read_u32::<LittleEndian>()? as f32;
        let acc_div = reader.read_u32::<LittleEndian>()? as f32;
        let acc_x = reader.read_i32::<LittleEndian>()? as f32;
        let acc_y = reader.read_i32::<LittleEndian>()? as f32;
        let acc_z = reader.read_i32::<LittleEndian>()? as f32;
        let accelerometer = SensorData3D {
            timestamp: acc_timestamp,
            x: (acc_x * acc_mul / acc_div) * 9.81 - self.accelerometer_bias.x,
            y: -(acc_y * acc_mul / acc_div) * 9.81 + self.accelerometer_bias.y,
            z: -(acc_z * acc_mul / acc_div) * 9.81 + self.accelerometer_bias.z,
        };
        Ok(GlassesEvent::AccGyro {
            accelerometer,
            gyroscope,
        })
    }
}

#[derive(Debug)]
struct Packet {
    category: u8,
    cmd_id: u8,
    data: Vec<u8>,
}

impl Default for Packet {
    fn default() -> Self {
        Self {
            category: 0,
            cmd_id: 0,
            data: vec![b'x'],
        }
    }
}

impl Packet {
    fn deserialize(data: &[u8]) -> Option<Packet> {
        if data[0] != 2 {
            return None;
        }
        let end = data.iter().position(|c| *c == 3)?;
        let inner = &data[1..end];
        let mut parts = inner.split(|c| *c == b':');
        let _empty = parts.next()?;
        let category = *parts.next()?.first()?;
        let cmd_id = *parts.next()?.first()?;
        let cmd_data = parts.next()?.into();
        // Next field is timestamp
        // Last field is CRC
        // TODO: maybe check CRC?
        Some(Packet {
            category,
            cmd_id,
            data: cmd_data,
        })
    }

    fn serialize(&self) -> Option<[u8; 0x40]> {
        let mut writer = std::io::Cursor::new([0u8; 0x40]);
        writer
            .write_all(&[2, b':', self.category, b':', self.cmd_id, b':'])
            .ok()?;
        writer.write_all(&self.data).ok()?;
        // Fake timestamp
        writer.write_all(b":0:").ok()?;
        let crc = crc32_adler(&writer.get_ref()[0..writer.position() as usize]);
        write!(writer, "{crc:>8x}").ok()?;
        writer.write_all(&[b':', 3]).ok()?;
        let result = writer.into_inner();
        Some(result)
    }
}
