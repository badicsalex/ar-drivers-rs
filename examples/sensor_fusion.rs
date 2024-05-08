// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

use ar_drivers::any_fusion;

fn main() {
    let mut fusion = any_fusion().unwrap(); // Declare conn as mutable

    let serial = fusion.glasses().serial().unwrap();
    println!("Got glasses, serial={}", serial);

    println!("");
    loop {
        fusion.update();
        let quaternion = fusion.attitude_quaternion();
        let frd = fusion.attitude_frd_deg();
        let inconsistency = fusion.inconsistency();

        println!("quaternion:\t{:10.7}", quaternion);
        println!("euler:\t{:10.7}", frd.transpose());

        println!("inconsistency:\t{:10.7}", inconsistency);
    }
}
