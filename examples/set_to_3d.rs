// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

use ar_drivers::{any_glasses, DisplayMode};
use clap::Parser;

/// Set AR glasses display mode
#[derive(clap::Parser, Debug)]
struct CliArgs {
    /// Display mode
    #[arg(value_enum, default_value_t = CliDisplayMode::Stereo)]
    mode: CliDisplayMode,

    /// Keep running after setting the mode.
    /// This is needed for some AR glasses to stay on in SBS mode.
    #[clap(long, short)]
    keep_running: bool,
}

#[derive(clap::ValueEnum, Debug, Clone)]
enum CliDisplayMode {
    /// Picture should be same for both eyes (simple full HD mode)
    #[value(name("2d"), alias("same-on-both"))]
    SameOnBoth,
    /// Set display to 3840*1080 or 3840*1200,
    /// where the left half is the left eye, the right half is the right eye
    #[value(name("3d"), alias("sbs"), alias("stereo"))]
    Stereo,
    /// Set display to half-SBS mode, which presents itself as 1920*1080 resolution,
    /// but actually scales down everything to 960x540,then upscales to 3840x1080
    // "sbs2" matches how half-SBS referred to in stereo3d filter in mpv player
    #[value(name("halfsbs"), alias("sbs2"), alias("half-stereo"))]
    HalfSBS,
    /// Set display to high refresh rate mode (120Hz in most cases)
    #[value(name("high-refresh-rate-2d"), alias("120hz"))]
    HighRefreshRate,
    /// Set display to high refresh rate SBS (90Hz on the Nreal Air)
    #[value(name("high-refresh-rate-3d"))]
    HighRefreshRateSBS,
}

fn main() {
    let args = CliArgs::parse();
    let mut glasses = any_glasses().unwrap();
    println!("Got glasses, serial={}", glasses.serial().unwrap());
    println!(
        "Display mode was: {:?}",
        glasses.get_display_mode().unwrap()
    );

    glasses
        .set_display_mode(match args.mode {
            CliDisplayMode::SameOnBoth => DisplayMode::SameOnBoth,
            CliDisplayMode::Stereo => DisplayMode::Stereo,
            CliDisplayMode::HalfSBS => DisplayMode::HalfSBS,
            CliDisplayMode::HighRefreshRate => DisplayMode::HighRefreshRate,
            CliDisplayMode::HighRefreshRateSBS => DisplayMode::HighRefreshRateSBS,
        })
        .unwrap();

    if args.keep_running {
        loop {
            glasses.read_event().unwrap();
        }
    }
}
