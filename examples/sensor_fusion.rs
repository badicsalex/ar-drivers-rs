// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

use ar_drivers::any_fusion;
use ar_drivers::Fusion;

fn main() {
    let mut fusion = any_fusion().unwrap(); // Declare conn as mutable

    let serial = fusion.glasses().serial().unwrap();
    println!("Got glasses, serial={}", serial);

    loop {
        fusion.update();
        let quaternion = fusion.attitude_quaternion();
        let euler = fusion.attitude_euler();

        println!("attitude:\n{}\neuler:\n{}", quaternion, euler);
    }
}
