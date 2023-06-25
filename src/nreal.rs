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
use tinyjson::JsonValue;

use crate::{ARGlasses, DisplayMode, Error, GlassesEvent, Result, SensorData3D};

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
            Some(b'2') => Ok(DisplayMode::Stereo),
            // "3&3D_1080"
            Some(b'3') => Ok(DisplayMode::Stereo),
            // "4&3D_1080#72"
            Some(b'4') => Ok(DisplayMode::Stereo),
            _ => Err(Error::Other("Unknown display mode")),
        }
    }

    fn set_display_mode(&mut self, display_mode: DisplayMode) -> Result<()> {
        let display_mode_byte = match display_mode {
            DisplayMode::SameOnBoth => b'1',
            // This could be 4 for 72Hz, but I don't trust that mode
            DisplayMode::Stereo => b'3',
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
        24.5f32.to_radians()
    }

    fn display_tilt(&self) -> f32 {
        -0.265
    }

    fn name(&self) -> &'static str {
        "Nreal Light"
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
    /// Connect to a specific Nreal device, based on the two USB fds
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
            HidApi::new()?.open(Self::MCU_PID, Self::MCU_VID)?,
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
}

struct Ov580 {
    device: HidDevice,
    config_json: JsonValue,
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
        };
        // Turn off IMU stream while reading config
        result.command(0x19, 0x0)?;
        result.read_config()?;
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
        // XXX: this may panic but at this point it's super unlikely.
        let accel_bias = &self.config_json["IMU"]["device_1"]["accel_bias"];
        let accel_bias_x = *accel_bias[0].get::<f64>().unwrap() as f32;
        let accel_bias_y = *accel_bias[1].get::<f64>().unwrap() as f32;
        let accel_bias_z = *accel_bias[2].get::<f64>().unwrap() as f32;
        let gyro_bias = &self.config_json["IMU"]["device_1"]["gyro_bias"];
        let gyro_bias_x = *gyro_bias[0].get::<f64>().unwrap() as f32;
        let gyro_bias_y = *gyro_bias[1].get::<f64>().unwrap() as f32;
        let gyro_bias_z = *gyro_bias[2].get::<f64>().unwrap() as f32;

        let gyro_timestamp = reader.read_u64::<LittleEndian>()? / 1000;
        let gyro_mul = reader.read_u32::<LittleEndian>()? as f32;
        let gyro_div = reader.read_u32::<LittleEndian>()? as f32;
        let gyro_x = reader.read_i32::<LittleEndian>()? as f32;
        let gyro_y = reader.read_i32::<LittleEndian>()? as f32;
        let gyro_z = reader.read_i32::<LittleEndian>()? as f32;
        let gyroscope = SensorData3D {
            timestamp: gyro_timestamp,
            x: (gyro_x * gyro_mul / gyro_div).to_radians() - gyro_bias_x,
            y: -(gyro_y * gyro_mul / gyro_div).to_radians() + gyro_bias_y,
            z: -(gyro_z * gyro_mul / gyro_div).to_radians() + gyro_bias_z,
        };

        let acc_timestamp = reader.read_u64::<LittleEndian>()? / 1000;
        let acc_mul = reader.read_u32::<LittleEndian>()? as f32;
        let acc_div = reader.read_u32::<LittleEndian>()? as f32;
        let acc_x = reader.read_i32::<LittleEndian>()? as f32;
        let acc_y = reader.read_i32::<LittleEndian>()? as f32;
        let acc_z = reader.read_i32::<LittleEndian>()? as f32;
        let accelerometer = SensorData3D {
            timestamp: acc_timestamp,
            x: (acc_x * acc_mul / acc_div) * 9.81 - accel_bias_x,
            y: -(acc_y * acc_mul / acc_div) * 9.81 + accel_bias_y,
            z: -(acc_z * acc_mul / acc_div) * 9.81 + accel_bias_z,
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
        let crc = crc32(&writer.get_ref()[0..writer.position() as usize]);
        write!(writer, "{crc:>8x}").ok()?;
        writer.write_all(&[b':', 3]).ok()?;
        let result = writer.into_inner();
        Some(result)
    }
}

// Code copied from rust-zip, but a similar code is also present in the
// javascript version of the firmware updater.
static CRCTABLE: [u32; 256] = [
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
    0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
    0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
    0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
    0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
    0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
    0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
    0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
    0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
    0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
    0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,
    0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683,
    0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
    0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
    0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
    0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
    0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
    0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21,
    0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
    0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
    0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
    0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94, 0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d,
];

fn crc32(buf: &[u8]) -> u32 {
    let mut r = 0xffffffffu32;
    for &byte in buf.iter() {
        let idx = byte ^ ((r & 0xff) as u8);
        r = (r >> 8) ^ CRCTABLE[idx as usize];
    }

    r ^ 0xffffffffu32
}
