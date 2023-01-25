# OpeNreal
[![Crates.io](https://img.shields.io/crates/v/openreal.svg)](https://crates.io/crates/openreal)
[![Docs.rs](https://docs.rs/openreal/badge.svg)](https://docs.rs/openreal)

OpeNreal is a simplified driver for Nreal Air (and possibly other) AR glasses.
It supports getting basic sensor data and setting up the display.

It only uses `rusb` for communication.

If you find Rust disgusting (and I wouldn't blame you), you should take a look
at the code and reimplement it in any other language, since it's so simple.

## Contribution

I appreciate reported bugs, feature requests and of course pull request.

I'd love to get my hands on an Nreal Lite, so if you have a spare one you'd
throw out, you could mail it to me instead :)

## License

Licensed under the MIT license

## Legal stuff

The protocol was obtained by reverse engineering Nreal's proprietary driver.
This is explicitly allowed in the EU for interworking purposes, it's probably
fair use everywhere else, but you should check your own country's laws.

The project is not affiliated with Nreal or related entities. 
