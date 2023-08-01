// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

//! Mad Gaze Glow AR glasses support. See [`MadGazeGlow`]
//! It only uses [`serialport`] for communication.

use std::{collections::VecDeque, io::Seek, thread::sleep, time::Duration};

use byteorder::{LittleEndian, ReadBytesExt};
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use serialport::{SerialPort, SerialPortType, UsbPortInfo};

use crate::{ARGlasses, DisplayMode, Error, GlassesEvent, Result, Side};

/*
        Sensor axes:
           /|               /|
          /                /
         /________________/
         |     |___|     |
         \_____/   \_____/

        Magnetometer:

                 Axis3
                 ^
                 |
                 |
                 |
                 o---> Axis1
                /
               /
              Axis2


        Accelerometer and Gyroscope:

        Axis2 <---o
                 /|
                / |
           Axis1  V
                  Axis3

        This is normalized to a standard right handed coordinate system.
*/

/// The main structure representing a connected Mad Gaze Glow glasses
pub struct MadGazeGlow {
    serial: SerialFraming,
    timestamp: u64,
    last_magnetometer_timestamp: u64,
    pending_events: VecDeque<GlassesEvent>,
}

impl ARGlasses for MadGazeGlow {
    fn serial(&mut self) -> Result<String> {
        String::from_utf8(self.serial.do_command(b"GSN", &[])?)
            .map_err(|_| "Invalid serial string".into())
    }

    fn read_event(&mut self) -> Result<GlassesEvent> {
        loop {
            if let Some(event) = self.pending_events.pop_front() {
                return Ok(event);
            }
            if self.last_magnetometer_timestamp + MAGNETOMETER_PERIOD < self.timestamp {
                self.update_ak09911()?;
                self.last_magnetometer_timestamp = self.timestamp;
            }
            self.update_bmi160()?;
        }
    }

    fn get_display_mode(&mut self) -> Result<DisplayMode> {
        let result = self.serial.do_command(b"G3D", &[])?;
        if result == [0] {
            Ok(DisplayMode::SameOnBoth)
        } else {
            Ok(DisplayMode::Stereo)
        }
    }

    fn set_display_mode(&mut self, display_mode: DisplayMode) -> Result<()> {
        let display_mode = match display_mode {
            DisplayMode::SameOnBoth => 0,
            DisplayMode::Stereo => 1,
            _ => return Err(Error::Other("Display mode not supported")),
        };
        let result = self.serial.do_command(b"S3D", &[display_mode])?;
        if result == [0] {
            Ok(())
        } else {
            Err(Error::Other("Invalid answer to the S3D command"))
        }
    }

    fn display_fov(&self) -> f32 {
        // The 23.5 degrees here is an actual measurement result
        //
        // However it actually does correspond somewhat to the advertised
        // 53 degree diagonal FOV:
        // degrees(atan(tan(radians(53))*(16/sqrt(16**2 + 9**2))))/2
        // == 24.5
        23.5f32.to_radians()
    }

    fn imu_to_display_matrix(&self, side: Side, ipd: f32) -> Isometry3<f64> {
        let ipd = ipd as f64
            * match side {
                Side::Left => -0.5,
                Side::Right => 0.5,
            };
        Translation3::new(ipd, 0.0, 0.0) * UnitQuaternion::from_euler_angles(0.12, 0.0, 0.0)
    }

    fn name(&self) -> &'static str {
        "Mad Gaze Glow"
    }

    fn display_delay(&self) -> u64 {
        // TODO: never actuallz calibrated
        15000
    }
}

const AK09911_ADDRESS: u8 = 12;
const AK09911_LSB_TO_UT: f32 = 4912.0 / 8190.0;

const BMI160_ADDRESS: u8 = 104;

// In useconds
const MAGNETOMETER_PERIOD: u64 = 50000;

impl MadGazeGlow {
    /// Find a connected Mad Gaze Glow device and connect to it.
    /// Only one instance should be alive at a time.
    pub fn new() -> Result<Self> {
        let mut result = Self {
            serial: SerialFraming::new()?,
            pending_events: Default::default(),
            timestamp: 0,
            last_magnetometer_timestamp: 0,
        };
        result.init_ak09911()?;
        result.init_bmi160()?;
        Ok(result)
    }

    /// Set the screen brightness. Allowed values are between 1-7
    pub fn set_sceen_brightness(&mut self, brightness: u8) -> Result<()> {
        let command = [brightness];
        let result = self.serial.do_command(b"SLB", &command)?;
        if result == [0] {
            Ok(())
        } else {
            Err(Error::Other("Invalid answer to the SLB command"))
        }
    }

    fn read_i2c(&mut self, address: u8, register: u8, length: u8) -> Result<Vec<u8>> {
        let command = [
            1, // Channel = 0, addres len = 1
            address, register, 0, // Second byte of the register
            length,
        ];
        let result = self.serial.do_command(b"I2R", &command)?;
        // TODO: We certainly hope here that this is our data.
        //       we could check the header of the result.
        // Also this is not the most performant way to do this but it doesn't matter here.
        Ok(result[5..].into())
    }

    fn write_i2c(&mut self, address: u8, register: u8, data: &[u8]) -> Result<()> {
        let mut command: Vec<u8> = [
            1, /* Channel = 0, addres len = 1 */
            address,
            register,
            0, // Second byte of the register
            data.len() as u8,
        ]
        .into();
        command.extend(data);
        let result = self.serial.do_command(b"I2W", &command)?;
        if result[4] == 0 {
            Ok(())
        } else {
            Err(Error::Other("Writing to I2C failed with an error."))
        }
    }

    fn init_ak09911(&mut self) -> Result<()> {
        let self_identification = self.read_i2c(AK09911_ADDRESS, 0, 2)?;
        if self_identification != [0x48, 0x05] {
            return Err(Error::Other("Wrong ak09911 self-ident."));
        }

        // Soft reset
        self.write_i2c(AK09911_ADDRESS, 0x32, &[0x01])?;
        sleep(Duration::from_millis(10));
        // Enable 100Hz Continous mode
        self.write_i2c(AK09911_ADDRESS, 0x31, &[0x08])?;
        Ok(())
    }

    fn update_ak09911(&mut self) -> Result<()> {
        let read_result = self.read_i2c(AK09911_ADDRESS, 0x10, 9)?;
        let mut reader = std::io::Cursor::new(read_result);
        reader.seek(std::io::SeekFrom::Start(1))?;

        let axis1 = reader.read_i16::<LittleEndian>()?;
        let axis2 = reader.read_i16::<LittleEndian>()?;
        let axis3 = reader.read_i16::<LittleEndian>()?;
        self.pending_events.push_back(GlassesEvent::Magnetometer {
            magnetometer: Vector3::new(
                -axis1 as f32 * AK09911_LSB_TO_UT,
                axis2 as f32 * AK09911_LSB_TO_UT,
                -axis3 as f32 * AK09911_LSB_TO_UT,
            ),
            timestamp: self.timestamp,
        });
        Ok(())
    }

    fn init_bmi160(&mut self) -> Result<()> {
        let self_identification = self.read_i2c(BMI160_ADDRESS, 0, 1)?;
        if self_identification != [0xd1] {
            return Err(Error::Other("Wrong BMI160 self-ident"));
        }

        // Soft reset
        self.write_i2c(BMI160_ADDRESS, 0x7e, &[0xb6])?;
        sleep(Duration::from_millis(10));
        // Enable accelerometer
        self.write_i2c(BMI160_ADDRESS, 0x7e, &[0x11])?;
        // Enable gyro
        self.write_i2c(BMI160_ADDRESS, 0x7e, &[0x15])?;
        // Enable acc and gyro in FIFO (headerless)
        self.write_i2c(BMI160_ADDRESS, 0x47, &[0xC0])?;
        Ok(())
    }

    fn update_bmi160(&mut self) -> Result<()> {
        // At more than 2 the data cannot be read at 100Hz
        const MAX_ENTRIES: u8 = 2;
        const ACC_UNIT: f32 = (2.0 * 9.80665) / 32768.0;
        const GYRO_UNIT: f32 = (2000.0 * std::f32::consts::PI / 180.0) / 32768.0;
        let read_result = self.read_i2c(BMI160_ADDRESS, 0x24, 12 * MAX_ENTRIES)?;
        let mut reader = std::io::Cursor::new(read_result);
        while reader.position() < 12 * MAX_ENTRIES as u64 {
            let gyro_axis1 = reader.read_i16::<LittleEndian>()?;
            let gyro_axis2 = reader.read_i16::<LittleEndian>()?;
            let gyro_axis3 = reader.read_i16::<LittleEndian>()?;
            let acc_axis1 = reader.read_i16::<LittleEndian>()?;
            let acc_axis2 = reader.read_i16::<LittleEndian>()?;
            let acc_axis3 = reader.read_i16::<LittleEndian>()?;

            if gyro_axis1 == -32768 {
                break;
            }
            self.pending_events.push_back(GlassesEvent::AccGyro {
                accelerometer: Vector3::new(
                    gyro_axis2 as f32 * GYRO_UNIT,
                    -gyro_axis3 as f32 * GYRO_UNIT,
                    -gyro_axis1 as f32 * GYRO_UNIT,
                ),
                gyroscope: Vector3::new(
                    acc_axis2 as f32 * ACC_UNIT,
                    -acc_axis3 as f32 * ACC_UNIT,
                    -acc_axis1 as f32 * ACC_UNIT,
                ),
                timestamp: self.timestamp,
            });
            // TODO: This is a big hack and should be read from the IMU
            // Then again, the IMU should fill its FIFO with this period,
            // and we are already running out of I2C or serial bandwidth
            self.timestamp += 10000;
        }
        Ok(())
    }
}

struct SerialFraming {
    port: Box<dyn SerialPort>,
}

impl SerialFraming {
    pub fn new() -> Result<Self> {
        let ports = serialport::available_ports()?;
        let ports: Vec<_> = ports
            .into_iter()
            .filter(|p| {
                matches!(
                    p.port_type,
                    SerialPortType::UsbPort(UsbPortInfo {
                        vid: 1204,
                        pid: 2,
                        ..
                    })
                )
            })
            .collect();
        if ports.is_empty() {
            return Err(Error::NotFound);
        }
        let port = serialport::new(&ports[0].port_name, 921600)
            .data_bits(serialport::DataBits::Eight)
            .stop_bits(serialport::StopBits::One)
            .parity(serialport::Parity::None)
            .timeout(Duration::from_millis(50))
            .open()?;
        port.clear(serialport::ClearBuffer::All)?;
        Ok(Self { port })
    }

    fn do_command(&mut self, cmd: &[u8], data: &[u8]) -> Result<Vec<u8>> {
        let cmd_data = Self::assemble_command(cmd, data);
        self.port.write_all(&cmd_data)?;
        loop {
            self.wait_for_sync_char()?;
            let mut header_buf = [0, 0, 0, 0];
            self.port.read_exact(&mut header_buf)?;
            let size = header_buf[3] as usize;
            let mut data_buf = [0; 256];
            self.port.read_exact(&mut data_buf[..size])?;
            if &header_buf[..3] == cmd {
                return Ok(data_buf[2..size - 3].into());
            }
        }
    }

    fn wait_for_sync_char(&mut self) -> Result<()> {
        loop {
            let mut buf = [0];
            let read_size = self.port.read(&mut buf)?;
            if read_size == 0 {
                return Err(Error::Other(
                    "Command read failure (zero bytes read) while waiting for ':'",
                ));
            }
            if buf[0] == b':' {
                break;
            }
        }
        Ok(())
    }

    fn assemble_command(cmd: &[u8], data: &[u8]) -> Vec<u8> {
        assert!(
            !data.contains(&b':'),
            "The protocol probably does not handle ':', so please don't"
        );

        [
            // The :CMD part
            &[b':'],
            cmd,
            // Packet length (including headers and footers after this byte)
            &[(data.len() + 5) as u8],
            // Fake session ID (we don't do multisessions)
            &[0xab, 0xcd],
            // The data itself
            data,
            // This should be a CRC16 checksum according to the java driver
            // But this works for some reason, and I take it.
            &[0, 0],
            // TODO: There's a workaround in the driver where if it finds a ':',
            // it replaces it to ';', and sets this byte to its index.
            // I instead hope that that byte will not be in the stream.
            &[0xff],
        ]
        .into_iter()
        .flatten()
        .copied()
        .collect()
    }
}
