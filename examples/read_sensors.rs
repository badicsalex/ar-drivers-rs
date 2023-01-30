// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

use ar_drivers::any_glasses;

fn main() {
    let mut glasses = any_glasses().unwrap();
    println!("Got glasses, serial={}", glasses.serial().unwrap());

    loop {
        let event = glasses.read_event().unwrap();
        println!("Event: {:#?}", event);
    }
}
