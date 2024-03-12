// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

use ar_drivers::{Connection, GetEuler, GetQuaternion, StartConnection};

fn main() {
    let code = StartConnection();

    println!("code {}", code);

    let mut lock = Connection::instance().fusion.lock().unwrap();

    let opt = lock.take();
    let mut fusion = opt.unwrap();

    let serial = fusion.glasses().serial().unwrap();
    println!("Got glasses, serial={}", serial);

    loop {
        unsafe {
            let quaternion = {
                let ptr = GetQuaternion() as *mut f32;
                Vec::from_raw_parts(ptr, 4, 4)
            };
            let euler = {
                let mut ptr = GetEuler() as *mut f32;
                Vec::from_raw_parts(ptr, 3, 3)
            };
            println!("attitude:\t{:?}\teuler:\t{:?}", quaternion, euler);
        }
    }
}
