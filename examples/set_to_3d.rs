// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

use ar_drivers::{DisplayMode, Glasses};

fn main() {
    let glasses = Glasses::new().unwrap();
    println!("Got glasses, serial={}", glasses.serial().unwrap());
    glasses.set_display_mode(DisplayMode::Stereo).unwrap();
}
