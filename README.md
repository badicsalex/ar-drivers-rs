# OpeNreal
[![Crates.io](https://img.shields.io/crates/v/openreal.svg)](https://crates.io/crates/openreal)
[![Docs.rs](https://docs.rs/openreal/badge.svg)](https://docs.rs/openreal)

Openreal is a simplified driver for Nreal Air (and possibly other) AR glasses.
It supports getting basic sensor data and setting up the display.

It only uses `rusb` for communication.

If you find Rust disgusting, like many do, you can still take a look at the code
and reimplement it in any other language, since it's so simple.

## License

Licensed under the MIT license

## Legal stuff

The protocol was obtained by reverse engineering Nreal's proprietary driver.
This is explicitly allowed in the EU for interworking purposes, it's probably
fair use everywhere else, but you should check your own country's laws.

The project is not affiliated with Nreal or related entities. 
