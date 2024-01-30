# AR driver library for Rust
[![Crates.io](https://img.shields.io/crates/v/ar-drivers.svg)](https://crates.io/crates/ar-drivers)
[![Docs.rs](https://docs.rs/ar-drivers/badge.svg)](https://docs.rs/ar-drivers)

This repository contains is a simplified Rust SDK for the following glasses:

* XREAL Air, Air 2, and Air 2 Pro
* XREAL Light
* Rokid Air
* Rokid Max
* Grawoow G530 (a.k.a. Metavision M53)
* Mad Gaze Glow

It supports getting basic sensor data and setting up the display.

While only the these glasses is supported right now, if I could get my hands on some other
ones, I'd happily support them too.

There's are two [somewhat detailed blog posts](https://voidcomputing.hu/blog/good-bad-ugly/) documenting [the various protocols](https://voidcomputing.hu/blog/worse-better-prettier/)
too. So if you're only interested in that, you won't even need to read Rust code :)

## 3D SBS mode switching

Some people only need programmatic mode switching. For that, all you need to do is:

Install dependencies (rust and libudev)

```
sudo apt install cargo libudev-dev libstdc++-12-dev
cargo update
```

Optional: add the udev scripts to your udev config, so the glasses are available to regular
users:

```
sudo cp udev/* /etc/udev/rules.d/
sudo udevadm control --reload
```

Run the code directly:

```
cargo run --example set_to_3d
```

Or build and then run:

```
cargo build --release --example set_to_3d
target/release/examples/set_to_3d
```

The executable is statically linked so you can copy it around, even to other PCs.


## Contribution

I appreciate reported bugs, feature requests and of course pull request.

## License

Licensed under the MIT license

## Legal stuff

Some parts of the protocols were obtained with reverse engineering.

Reverse engineering explicitly allowed in the EU for interworking purposes.
It's probably fair use everywhere else, but you should check your own
country's laws.

The project is not affiliated with any of the manufacturers or their related entities.
