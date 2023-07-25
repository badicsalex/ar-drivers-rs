// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

// Based on code by thejackimonster
// See https://gitlab.com/TheJackiMonster/nrealAirLinuxDriver

//! Nreal Light AR glasses support. See [`NrealLight`]
//! It only uses [`hidapi`] for communication.
//!
//! **Important note**: The NReal Light requires constant heartbeats in 3D SBS mode,
//! or else it switches the screen off. This heartbeat is sent periodically when
//! [`NrealLight::read_event`] is called, so be sure to constantly call that function (at least once
//! every half a second or so)

use std::collections::VecDeque;

use byteorder::{LittleEndian, ReadBytesExt};
use hidapi::{HidApi, HidDevice};
use tinyjson::JsonValue;

use crate::{
    util::{crc32_adler, Vector3},
    ARGlasses, DisplayMode, Error, GlassesEvent, Result, SensorData3D,
};

/// The main structure representing a connected Nreal Light glasses
pub struct NrealAir {
    device: HidDevice,
    pending_packets: VecDeque<McuPacket>,
    imu_device: ImuDevice,
}

const COMMAND_TIMEOUT: i32 = 250;
const IMU_TIMEOUT: i32 = 250;

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
        match result.first() {
            // Mirror 60Hz
            Some(1) => Ok(DisplayMode::SameOnBoth),
            // SBS 60Hz
            Some(3) => Ok(DisplayMode::Stereo),
            // SBS 72Hz
            Some(4) => Ok(DisplayMode::HighRefreshRate),
            // Mirror 72Hz
            Some(5) => Ok(DisplayMode::SameOnBoth),
            _ => Err(Error::Other("Unknown display mode")),
        }
    }

    fn set_display_mode(&mut self, display_mode: DisplayMode) -> Result<()> {
        let display_mode_byte = match display_mode {
            DisplayMode::SameOnBoth => 1,
            DisplayMode::HalfSBS => return Err(Error::Other("Display mode not supported")),
            DisplayMode::Stereo => 3,
            DisplayMode::HighRefreshRate => 4,
        };
        let result = self.run_command(McuPacket {
            cmd_id: 0x08,
            data: vec![display_mode_byte],
        })?;

        if result.first() == Some(&display_mode_byte) {
            Ok(())
        } else {
            Err(Error::Other("Display mode setting unsuccessful"))
        }
    }

    // TODO
    fn display_fov(&self) -> f32 {
        // 24.0° is the advertised FOV
        // According to measurements, it is even a bit better
        25.0f32.to_radians()
    }

    fn display_tilt(&self) -> f32 {
        // Apparently it is very well aligned.
        0.0
    }

    fn name(&self) -> &'static str {
        "Nreal Air"
    }
}

impl NrealAir {
    /// Vendor ID of the NReal Air's components
    pub const VID: u16 = 0x3318;
    /// Product ID of the NReal Air's components
    pub const PID: u16 = 0x0424;

    /// Find a connected Nreal Light device and connect to it. (And claim the USB interface)
    /// Only one instance can be alive at a time
    #[cfg(not(target_os = "android"))]
    pub fn new() -> Result<Self> {
        Self::new_common(
            open_vid_pid_endpoint(Self::VID, Self::PID, 4)?,
            ImuDevice::new()?,
        )
    }
    fn new_common(device: HidDevice, imu_device: ImuDevice) -> Result<Self> {
        let mut result = Self {
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
    gyro_bias: Vector3,
    accelerometer_bias: Vector3,
}

impl ImuDevice {
    #[cfg(target_os = "android")]
    pub fn new(fd: isize) -> Result<Self> {
        Self::new_device(HidApi::new_without_enumerate()?.wrap_sys_device(fd, -1)?)
    }

    #[cfg(not(target_os = "android"))]
    pub fn new() -> Result<Self> {
        Self::new_device(open_vid_pid_endpoint(NrealAir::VID, NrealAir::PID, 3)?)
    }
    fn new_device(device: HidDevice) -> Result<Self> {
        let mut result = Self {
            device,
            config_json: JsonValue::Null,
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
        let cfg = &self.config_json["IMU"]["device_1"];
        self.accelerometer_bias = Self::parse_vector(&cfg["accel_bias"]);
        self.gyro_bias = Self::parse_vector(&cfg["gyro_bias"]);
        Ok(())
    }

    fn parse_vector(json: &JsonValue) -> Vector3 {
        Vector3 {
            x: *json[0].get::<f64>().unwrap() as f32,
            y: *json[1].get::<f64>().unwrap() as f32,
            z: *json[2].get::<f64>().unwrap() as f32,
        }
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
        let gyroscope = SensorData3D {
            timestamp,
            // The bias fields do not correspond to the raw fields, but for some reason
            // this looks like the correct zero.
            x: -(gyro_x * gyro_mul / gyro_div).to_radians() - self.gyro_bias.x,
            y: (gyro_z * gyro_mul / gyro_div).to_radians() + self.gyro_bias.y,
            z: (gyro_y * gyro_mul / gyro_div).to_radians() + self.gyro_bias.z,
        };

        let acc_mul = reader.read_u16::<LittleEndian>()? as f32;
        let acc_div = reader.read_u32::<LittleEndian>()? as f32;
        let acc_x = reader.read_i24::<LittleEndian>()? as f32;
        let acc_y = reader.read_i24::<LittleEndian>()? as f32;
        let acc_z = reader.read_i24::<LittleEndian>()? as f32;
        let accelerometer = SensorData3D {
            timestamp,
            // The bias fields do not correspond to the raw fields, but for some reason
            // this looks like the correct zero.
            x: -(acc_x * acc_mul / acc_div) * 9.81 - self.accelerometer_bias.x,
            y: (acc_z * acc_mul / acc_div) * 9.81 + self.accelerometer_bias.y,
            z: (acc_y * acc_mul / acc_div) * 9.81 + self.accelerometer_bias.z,
        };
        eprintln!("{accelerometer:?}\n{}", self.accelerometer_bias.z);
        // TODO: magnetometer. It's in the same format, but it's non-trivially
        //       rotated.
        // TODO: Check checksum
        Ok(GlassesEvent::AccGyro {
            accelerometer,
            gyroscope,
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

fn open_vid_pid_endpoint(vid: u16, pid: u16, interface: i32) -> Result<HidDevice> {
    let hidapi = HidApi::new()?;
    for device in hidapi.device_list() {
        if device.vendor_id() == vid
            && device.product_id() == pid
            && device.interface_number() == interface
        {
            return Ok(device.open_device(&hidapi)?);
        }
    }
    Err(Error::NotFound)
}
