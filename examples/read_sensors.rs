// Copyright (C) 2023, Alex Badics
// This file is part of openreal
// Licensed under the MIT license. See LICENSE file in the project root for details.

use openreal::{Glasses, GlassesEvent};

fn main() {
    let glasses = Glasses::new().unwrap();
    println!("Got glasses, serial={}", glasses.serial().unwrap());

    loop {
        let event = glasses.read_event().unwrap();
        if matches!(event, GlassesEvent::Magnetometer(_)) {
            println!("Event: {:?}", event);
        }
    }
}
