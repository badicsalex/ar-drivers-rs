// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

use std::time::{Duration, Instant};

use ar_drivers::{any_glasses, GlassesEvent};
use nalgebra::Vector3;

fn main() {
    let mut glasses = any_glasses().unwrap();
    println!("Got glasses, serial={}", glasses.serial().unwrap());

    let mut last_print = Instant::now();
    let mut acc_sum = Vector3::zeros();
    let mut gyro_sum = Vector3::zeros();
    let mut n = 0;
    loop {
        if let GlassesEvent::AccGyro {
            accelerometer,
            gyroscope,
            ..
        } = glasses.read_event().unwrap()
        {
            acc_sum += accelerometer;
            gyro_sum += gyroscope;
            n += 1;
            if last_print.elapsed() > Duration::from_secs(1) {
                println!(
                    "Acceleration:\n{}Gyro:\n{}",
                    acc_sum * (1.0 / n as f32),
                    gyro_sum * (1.0 / n as f32)
                );
                last_print = Instant::now();
                acc_sum = Default::default();
                gyro_sum = Default::default();
                n = 0;
            }
        }
    }
}
