// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

// Based on code by thejackimonster
// See https://gitlab.com/TheJackiMonster/nrealAirLinuxDriver

//! Nreal Air AR glasses support. See [`NrealAir`]
//! It only uses [`hidapi`] for communication.

use std::collections::VecDeque;

use byteorder::{LittleEndian, ReadBytesExt};
use hidapi::{HidApi, HidDevice};
use nalgebra::{Isometry3, Matrix3, Quaternion, Translation3, UnitQuaternion, Vector3};
use tinyjson::JsonValue;

use crate::{
    util::crc32_adler, ARGlasses, DisplayMatrices, DisplayMode, Error, GlassesEvent, Result, Side,
};

/// The main structure representing a connected Nreal Air glasses
pub struct NrealAir {
    model: AirModel,
    device: HidDevice,
    pending_packets: VecDeque<McuPacket>,
    imu_device: ImuDevice,
}

const COMMAND_TIMEOUT: i32 = 1000;
const IMU_TIMEOUT: i32 = 250;

const NREAL_VID: u16 = 0x3318;
const AIR_PID: u16 = 0x0424;
const AIR_2_PID: u16 = 0x0428;
const AIR_2_PRO_PID: u16 = 0x0432;

/// Describes the particular Air model.
pub enum AirModel {
    /// XREAL Air (original)
    Air,
    /// XREAL Air 2
    Air2,
    /// XREAL Air 2 Pro
    Air2Pro,
}

impl TryFrom<u16> for AirModel {
    type Error = Error;

    fn try_from(val: u16) -> Result<AirModel> {
        match val {
            AIR_PID => Ok(AirModel::Air),
            AIR_2_PID => Ok(AirModel::Air2),
            AIR_2_PRO_PID => Ok(AirModel::Air2Pro),
            _ => Err(Error::Other("unsupported XREAL product")),
        }
    }
}

impl ARGlasses for NrealAir {
    fn serial(&mut self) -> Result<String> {
        let mut result = self.run_command(McuPacket {
            cmd_id: 0x15,
            ..Default::default()
        })?;
        result.remove(0);
        String::from_utf8(result).map_err(|_| Error::Other("Serial number was not utf-8"))
    }

    fn read_event(&mut self) -> Result<GlassesEvent> {
        if let Some(event) = self.read_mcu_packet()? {
            Ok(event)
        } else {
            self.imu_device.read_packet()
        }
    }

    fn get_display_mode(&mut self) -> Result<DisplayMode> {
        let result = self.run_command(McuPacket {
            cmd_id: 0x7,
            ..Default::default()
        })?;
        match result.get(1) {
            // Mirror 60Hz
            Some(1) => Ok(DisplayMode::SameOnBoth),
            // SBS 60Hz
            Some(3) => Ok(DisplayMode::Stereo),
            // SBS 72Hz
            Some(4) => Ok(DisplayMode::HighRefreshRate),
            // Mirror 72Hz
            Some(5) => Ok(DisplayMode::SameOnBoth),
            // Horizontally stretched SBS, 60Hz
            Some(8) => Ok(DisplayMode::HalfSBS),
            // SBS 90Hz
            Some(9) => Ok(DisplayMode::HighRefreshRate),
            // Mirror 90Hz
            Some(10) => Ok(DisplayMode::HighRefreshRate),
            // Mirror 120Hz
            Some(11) => Ok(DisplayMode::HighRefreshRate),
            _ => Err(Error::Other("Unknown display mode")),
        }
    }

    fn set_display_mode(&mut self, display_mode: DisplayMode) -> Result<()> {
        let display_mode_byte = match display_mode {
            DisplayMode::SameOnBoth => 1,
            DisplayMode::HalfSBS => 8,
            DisplayMode::Stereo => 3,
            DisplayMode::HighRefreshRate => 11,
            DisplayMode::HighRefreshRateSBS => 9,
        };
        let result = self.run_command(McuPacket {
            cmd_id: 0x08,
            data: vec![display_mode_byte],
        })?;

        if result.first() == Some(&0) {
            Ok(())
        } else {
            Err(Error::Other("Display mode setting unsuccessful"))
        }
    }

    // TODO
    fn display_fov(&self) -> f32 {
        // This is a judgement call. The displays have a non-trivial distortion,
        // so this value is a bit much in hte middle, a bit too low on the borders.
        24.0f32.to_radians()
    }

    fn imu_to_display_matrix(&self, side: Side, ipd: f32) -> Isometry3<f64> {
        let side_multiplier = match side {
            Side::Left => -0.5,
            Side::Right => 0.5,
        };
        Translation3::new(ipd as f64 * side_multiplier, 0.0, 0.0)
            * UnitQuaternion::from_euler_angles(
                // Apparently there is no noticable tilt
                0.0,
                Self::DISPLAY_DIVERGENCE * side_multiplier,
                0.0,
            )
    }

    fn display_matrices(&self) -> Result<(DisplayMatrices, DisplayMatrices)> {
        self.imu_device.displays.clone().ok_or(Error::NotFound)
    }

    fn display_delay(&self) -> u64 {
        7000
    }

    fn name(&self) -> &'static str {
        match self.model {
            AirModel::Air => "XREAL Air",
            AirModel::Air2 => "XREAL Air 2",
            AirModel::Air2Pro => "XREAL Air 2 Pro",
        }
    }
}

impl NrealAir {
    /// Vendor ID of the NReal Air's components
    pub const VID: u16 = NREAL_VID;
    /// Product ID of the NReal Air 1's components
    #[deprecated]
    pub const PID: u16 = AIR_PID;

    const DISPLAY_DIVERGENCE: f64 = 0.017;

    /// Connect to a specific glasses, based on the
    /// Mainly made to work around android permission issues
    #[cfg(target_os = "android")]
    pub fn new(fd: isize) -> Result<Self> {
        let device = HidApi::new_without_enumerate()?.wrap_sys_device(fd, 4)?;
        let pid = device.get_device_info()?.product_id();
        let model = AirModel::try_from(pid)?;
        Self::new_common(model, device, ImuDevice::new(fd)?)
    }

    /// Find a connected Nreal Air device and connect to it. (And claim the USB interface)
    /// Only one instance can be alive at a time
    #[cfg(not(target_os = "android"))]
    pub fn new() -> Result<Self> {
        let (model, device) = open_nreal_endpoint(4)?;
        Self::new_common(model, device, ImuDevice::new()?)
    }
    fn new_common(model: AirModel, device: HidDevice, imu_device: ImuDevice) -> Result<Self> {
        let mut result = Self {
            model,
            device,
            pending_packets: Default::default(),
            imu_device,
        };
        // Quick check
        result.serial()?;
        Ok(result)
    }

    /// Returns the calibration data stored on the Glasses. No transformation
    /// is done on the data, except for Json Parsing.
    pub fn get_config_json(&self) -> &JsonValue {
        &self.imu_device.config_json
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
            McuPacket {
                cmd_id: 0x6c05,
                data,
            } => Some(GlassesEvent::KeyPress(data[0] - 1)),
            // NOTE: maybe we should retry in these cases instead of basically reporting timeout,
            //       but we will be called again soon enough.
            McuPacket {
                cmd_id: 0x6c09,
                data: _data,
            } => {
                // TODO: optional logging in the crate
                // eprintln!("Got error: {}", String::from_utf8(_data).unwrap());
                None
            }
            _ => None,
        })
    }

    fn read_packet(&mut self, timeout: i32) -> Result<Option<McuPacket>> {
        let mut result = [0u8; 0x40];
        let packet_size = self.device.read_timeout(&mut result, timeout)?;
        if packet_size == 0 {
            Ok(None)
        } else {
            Ok(Some(
                McuPacket::deserialize(&result).ok_or(Error::Other("Malformed packet received"))?,
            ))
        }
    }

    fn run_command(&mut self, command: McuPacket) -> Result<Vec<u8>> {
        self.device.write(
            &command
                .serialize()
                .ok_or(Error::Other("Packet serialization failed"))?,
        )?;

        for _ in 0..64 {
            let packet = self
                .read_packet(COMMAND_TIMEOUT)?
                .ok_or(Error::PacketTimeout)?;
            if packet.cmd_id == command.cmd_id {
                return Ok(packet.data);
            }
            self.pending_packets.push_back(packet);
        }
        Err(Error::Other("Received too many unrelated packets"))
    }
}

struct ImuDevice {
    device: HidDevice,
    config_json: JsonValue,
    displays: Option<(DisplayMatrices, DisplayMatrices)>,
    gyro_bias: Vector3<f32>,
    accelerometer_bias: Vector3<f32>,
}

impl ImuDevice {
    #[cfg(target_os = "android")]
    pub fn new(fd: isize) -> Result<Self> {
        Self::new_device(HidApi::new_without_enumerate()?.wrap_sys_device(fd, 3)?)
    }

    #[cfg(not(target_os = "android"))]
    pub fn new() -> Result<Self> {
        let (_, device) = open_nreal_endpoint(3)?;
        Self::new_device(device)
    }
    fn new_device(device: HidDevice) -> Result<Self> {
        let mut result = Self {
            device,
            config_json: JsonValue::Null,
            displays: None,
            gyro_bias: Default::default(),
            accelerometer_bias: Default::default(),
        };
        // Turn off IMU stream while reading config
        result.command(0x19, &[0x0])?;
        result.read_config()?;
        result.parse_config()?;
        // Turn IMU stream back on
        result.command(0x19, &[0x1])?;

        Ok(result)
    }

    fn read_config(&mut self) -> Result<()> {
        let len = u32::from_le_bytes(self.command(0x14, &[])?.try_into().unwrap());
        let mut config = Vec::new();
        while config.len() < len as usize {
            let mut config_part = self.command(0x15, &[])?;
            config.append(&mut config_part);
        }
        let config_as_str = String::from_utf8(config)
            .map_err(|_| Error::Other("Invalid glasses config (not utf-8)"))?;
        self.config_json = config_as_str
            .parse()
            .map_err(|_| Error::Other("Invalid glasses config format (JSON parse error)"))?;
        Ok(())
    }

    fn parse_config(&mut self) -> Result<()> {
        // XXX: This will panic if config is not in expected format.
        //      should probably return Err() instead.
        self.displays = Self::parse_display_descriptors(&self.config_json["display"]);
        let cfg = &self.config_json["IMU"]["device_1"];
        self.accelerometer_bias = Self::parse_vector(&cfg["accel_bias"]).map(|c| c as f32);
        self.gyro_bias = Self::parse_vector(&cfg["gyro_bias"]).map(|c| c as f32);
        Ok(())
    }

    fn parse_display_descriptors(json: &JsonValue) -> Option<(DisplayMatrices, DisplayMatrices)> {
        let resolution = &json["resolution"];
        let resolution = (
            *resolution[0].get::<f64>().unwrap() as u32,
            *resolution[1].get::<f64>().unwrap() as u32,
        );

        let side_descriptor =
            |p_cfg: &JsonValue, q_cfg: &JsonValue, k_cfg: &JsonValue| -> DisplayMatrices {
                let translation = Self::parse_vector(p_cfg);
                let rotation = UnitQuaternion::from_quaternion(Self::parse_quaternion(q_cfg));
                DisplayMatrices {
                    intrinsic_matrix: Self::parse_matrix3(k_cfg),
                    resolution,
                    isometry: Translation3::from(translation) * rotation,
                }
            };
        let mut left = side_descriptor(
            &json["target_p_left_display"],
            &json["target_q_left_display"],
            &json["k_left_display"],
        );
        let mut right = side_descriptor(
            &json["target_p_right_display"],
            &json["target_q_right_display"],
            &json["k_right_display"],
        );
        // The calibration seems to be based on a reference point near the right lens.
        // We will center the translation component between the displays.
        let mean = (left.isometry.translation.vector + right.isometry.translation.vector) * 0.5;
        left.isometry.translation.vector -= mean;
        right.isometry.translation.vector -= mean;
        Some((left, right))
    }

    fn parse_vector(json: &JsonValue) -> Vector3<f64> {
        Vector3::new(
            *json[0].get::<f64>().unwrap(),
            *json[1].get::<f64>().unwrap(),
            *json[2].get::<f64>().unwrap(),
        )
    }

    fn parse_quaternion(json: &JsonValue) -> Quaternion<f64> {
        Quaternion::new(
            *json[3].get::<f64>().unwrap(),
            *json[0].get::<f64>().unwrap(),
            *json[1].get::<f64>().unwrap(),
            *json[2].get::<f64>().unwrap(),
        )
    }

    fn parse_matrix3(json: &JsonValue) -> Matrix3<f64> {
        let vals = json
            .get::<Vec<_>>()
            .unwrap()
            .iter()
            .map(|v| *v.get::<f64>().unwrap())
            .collect::<Vec<f64>>();
        Matrix3::from_row_slice(&vals)
    }

    fn command(&self, cmd_id: u8, data: &[u8]) -> Result<Vec<u8>> {
        self.device.write(
            &ImuPacket {
                cmd_id,
                data: data.into(),
            }
            .serialize()
            .ok_or(Error::Other("Couldn't get acknowledgement to command"))?,
        )?;
        for _ in 0..64 {
            let mut data = [0u8; 0x40];
            let result_size = self.device.read_timeout(&mut data, IMU_TIMEOUT)?;
            if result_size == 0 {
                return Err(Error::PacketTimeout);
            }

            if let Some(result) = ImuPacket::deserialize(&data) {
                return Ok(result.data);
            }
        }
        Err(Error::Other("Couldn't get acknowledgement to command"))
    }

    pub fn read_packet(&mut self) -> Result<GlassesEvent> {
        loop {
            let mut packet_data = [0u8; 0x80];
            let data_size = self.device.read_timeout(&mut packet_data, IMU_TIMEOUT)?;
            if data_size == 0 {
                return Err(Error::PacketTimeout);
            }

            if packet_data[0] == 1 && packet_data[1] == 2 {
                return self.parse_report(&packet_data);
            };
            // Else try again
        }
    }

    fn parse_report(&mut self, packet_data: &[u8]) -> Result<GlassesEvent> {
        // TODO: This skips over a 2 byte temperature field that may be useful.
        let mut reader = std::io::Cursor::new(&packet_data[4..]);

        let timestamp = reader.read_u64::<LittleEndian>()? / 1000;
        let gyro_mul = reader.read_u16::<LittleEndian>()? as f32;
        let gyro_div = reader.read_u32::<LittleEndian>()? as f32;
        let gyro_x = reader.read_i24::<LittleEndian>()? as f32;
        let gyro_y = reader.read_i24::<LittleEndian>()? as f32;
        let gyro_z = reader.read_i24::<LittleEndian>()? as f32;
        let gyroscope = Vector3::new(
            // The bias fields do not correspond to the raw fields, but for some reason
            // this looks like the correct zero.
            -(gyro_x * gyro_mul / gyro_div).to_radians() - self.gyro_bias.x,
            (gyro_z * gyro_mul / gyro_div).to_radians() + self.gyro_bias.y,
            (gyro_y * gyro_mul / gyro_div).to_radians() + self.gyro_bias.z,
        );

        let acc_mul = reader.read_u16::<LittleEndian>()? as f32;
        let acc_div = reader.read_u32::<LittleEndian>()? as f32;
        let acc_x = reader.read_i24::<LittleEndian>()? as f32;
        let acc_y = reader.read_i24::<LittleEndian>()? as f32;
        let acc_z = reader.read_i24::<LittleEndian>()? as f32;
        let accelerometer = Vector3::new(
            // The bias fields do not correspond to the raw fields, but for some reason
            // this looks like the correct zero.
            -(acc_x * acc_mul / acc_div) * 9.81 - self.accelerometer_bias.x,
            (acc_z * acc_mul / acc_div) * 9.81 + self.accelerometer_bias.y,
            (acc_y * acc_mul / acc_div) * 9.81 + self.accelerometer_bias.z,
        );
        // TODO: magnetometer. It's in the same format, but it's non-trivially
        //       rotated.
        // TODO: Check checksum
        Ok(GlassesEvent::AccGyro {
            accelerometer,
            gyroscope,
            timestamp,
        })
    }
}

#[derive(Debug, Default)]
struct McuPacket {
    cmd_id: u16,
    data: Vec<u8>,
}

#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
struct McuRawPacket {
    head: u8,
    checksum: u32,
    length: u16,
    request_id: u32,
    timestamp: u32,
    cmd_id: u16,
    reserved: [u8; 5],
    data: [u8; 42],
}

unsafe impl bytemuck::Zeroable for McuRawPacket {}
unsafe impl bytemuck::Pod for McuRawPacket {}

impl McuPacket {
    fn deserialize(data: &[u8; 0x40]) -> Option<McuPacket> {
        let raw_packet: &McuRawPacket = bytemuck::cast_ref(data);
        if raw_packet.head != 0xfd {
            return None;
        }
        // TODO: maybe check CRC?
        Some(McuPacket {
            cmd_id: raw_packet.cmd_id,
            data: raw_packet.data[0..(raw_packet.length as usize - 17)].into(),
        })
    }

    fn serialize(&self) -> Option<[u8; 0x40]> {
        let mut data = [0u8; 42];
        data[0..self.data.len()].copy_from_slice(&self.data);
        let mut raw_packet = McuRawPacket {
            head: 0xfd,
            checksum: 0,
            length: self.data.len() as u16 + 17,
            request_id: 0x1337,
            timestamp: 0x0,
            cmd_id: self.cmd_id,
            reserved: Default::default(),
            data,
        };
        raw_packet.checksum =
            crc32_adler(&bytemuck::bytes_of(&raw_packet)[5..(5 + raw_packet.length as usize)]);
        Some(bytemuck::cast(raw_packet))
    }
}

#[derive(Debug, Default)]
struct ImuPacket {
    cmd_id: u8,
    data: Vec<u8>,
}

#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
struct ImuRawPacket {
    head: u8,
    checksum: u32,
    length: u16,
    cmd_id: u8,
    data: [u8; 56],
}

unsafe impl bytemuck::Zeroable for ImuRawPacket {}
unsafe impl bytemuck::Pod for ImuRawPacket {}

impl ImuPacket {
    fn deserialize(data: &[u8; 0x40]) -> Option<ImuPacket> {
        let raw_packet: &ImuRawPacket = bytemuck::cast_ref(data);
        if raw_packet.head != 0xaa {
            return None;
        }
        // TODO: maybe check CRC?
        Some(ImuPacket {
            cmd_id: raw_packet.cmd_id,
            data: raw_packet.data[0..(raw_packet.length as usize - 3)].into(),
        })
    }

    fn serialize(&self) -> Option<[u8; 0x40]> {
        let mut data = [0u8; 56];
        data[0..self.data.len()].copy_from_slice(&self.data);
        let mut raw_packet = ImuRawPacket {
            head: 0xaa,
            checksum: 0,
            length: self.data.len() as u16 + 3,
            cmd_id: self.cmd_id,
            data,
        };
        raw_packet.checksum =
            crc32_adler(&bytemuck::bytes_of(&raw_packet)[5..(5 + raw_packet.length as usize)]);
        Some(bytemuck::cast(raw_packet))
    }
}

#[cfg(not(target_os = "android"))]
fn open_nreal_endpoint(interface: i32) -> Result<(AirModel, HidDevice)> {
    let hidapi = HidApi::new()?;
    for device in hidapi.device_list() {
        if device.vendor_id() == NREAL_VID && device.interface_number() == interface {
            let model = match AirModel::try_from(device.product_id()) {
                Ok(m) => m,
                Err(_) => continue,
            };
            return Ok((model, device.open_device(&hidapi)?));
        }
    }
    Err(Error::NotFound)
}
