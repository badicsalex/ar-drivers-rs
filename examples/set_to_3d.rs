// Copyright (C) 2023, Alex Badics
// This file is part of openreal
// Licensed under the MIT license. See LICENSE file in the project root for details.

use openreal::{DisplayMode, Glasses};

fn main() {
    let glasses = Glasses::new().unwrap();
    println!("Got glasses, serial={}", glasses.serial().unwrap());
    glasses.set_display_mode(DisplayMode::Stereo).unwrap();
}
