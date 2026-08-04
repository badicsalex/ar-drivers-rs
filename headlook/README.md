# HeadLook

Head-tracked mouse control for **XREAL Air / Air 2 / Air 2 Pro / Air 2 Ultra** (and other
glasses supported by [`ar-drivers`](https://github.com/badicsalex/ar-drivers-rs)) on **Windows
(x86_64)**. Turn your head to look around in first-person shooters and similar games — no
head-mounted display required, just the IMU in the glasses.

This is a binary crate that lives inside the `ar-drivers-rs` repository (under `headlook/`) so it
reuses the existing `ar-drivers` driver for device detection and IMU reading.

## How it works

1. The glasses are detected and connected via `ar-drivers` (`any_glasses()`).
2. IMU samples (accelerometer + gyroscope, ~200–1000 Hz) are fed into a **sensor-fusion filter**
   (complementary/Mahony or Madgwick) that estimates head orientation. Gyroscope bias is learned
   automatically while the head is still, which removes slow cursor drift.
3. The head's **yaw** (left/right) and **pitch** (up/down) are converted into *relative* mouse
   motion, scaled by a configurable sensitivity and smoothed. Sub-pixel remainders are
   accumulated so movement stays precise.
4. Motion is emitted with `SendInput` (native Win32) at a coalesced rate for low latency and
   low overhead.

## Features

- **Global hotkey** `Ctrl+A+R` toggles head tracking on/off (configurable).
- Runs in the background with a **system tray** icon and menu.
- **Shooting safety**: holding the **left mouse button** (or any configured extra key, e.g. RMB)
  completely pauses head-to-mouse translation.
- Configurable **sensitivity, deadzone, smoothing, vertical lock, non-linear aim curve**.
- Automatic **reconnect** if the glasses are unplugged, plus graceful degradation when
  disconnected.
- `--list` to detect the glasses, `--dry-run` (no mouse output) for tuning, `--no-tray` console
  mode.

## Requirements

- Windows 10/11, x86_64.
- XREAL Air series glasses connected over USB-C. For the IMU to stream, the glasses must be in
  **3D/SBS mode** (so the host sees the device). The program does not switch display modes —
  set it once in the glasses' companion app or via `ar-drivers` if needed.
- **Run as Administrator** if the target game runs elevated. `SendInput` is subject to
  UIPI: an input emitted from a non-elevated process cannot reach an elevated window.

## Build

You need the Rust stable toolchain (MSVC target). From the repo root:

```powershell
# Build the HeadLook binary only
cargo build -p headlook --target x86_64-pc-windows-msvc --release

# The executable is at:
#   target/x86_64-pc-windows-msvc/release/headlook.exe
```

> The `ar-drivers` dependency is built with only the `nreal` feature (no `rusb`/`serialport`),
> which keeps the build simple on Windows. To support *every* glasses type, build with
> `--features all-glasses`.

GitHub Actions (`.github/workflows/ci.yml`) builds and tests the project on Windows automatically
on every push/PR, so you can verify a build without a local Windows machine.

## Usage

1. Connect the glasses (3D/SBS mode).
2. Run `headlook.exe`. A tray icon appears; by default tracking starts immediately.
3. Press **Ctrl+A+R** to toggle tracking.
4. Right-click the tray icon for: toggle, vertical lock, recenter, re-calibrate gyro bias,
   open config, reload config, exit.
5. To aim/shoot without the view moving, hold the left mouse button (or your configured pause
   keys).

### Configuration

On first run, `headlook.toml` is written next to the executable (or in
`%APPDATA%/HeadLook/headlook.toml`). Edit it, then choose **Reload config** from the tray menu
— most settings apply live within ~2 seconds. See `headlook.toml` in this folder for the full,
documented set of options. Highlights:

```toml
[tracking]
hotkey = "Ctrl+A+R"        # toggle combo (e.g. "Ctrl+Shift+H", "Alt+G")
enabled_on_start = true
sensitivity = 14.0         # pixels per degree
deadzone_dps = 1.5         # ignore slower motion (kills jitter)
vertical_lock = false      # only track yaw
invert_pitch = false
invert_yaw = false
accel_exponent = 1.6       # >1 = slower near center, faster at edges
pause_keys = ["RMB"]       # extra keys that pause tracking while held

[filter]
mode = "complementary"     # "complementary" (Mahony) or "madgwick"
smoothing_tau = 0.012      # output smoothing time constant (s); 0 = off

[mouse]
emit_rate = 500            # mouse event rate (Hz); 0 = every IMU sample
```

## Architecture

| Module            | Responsibility                                                        |
|-------------------|-----------------------------------------------------------------------|
| `config.rs`       | Load/save/validate `headlook.toml`, defaults, sanitization.          |
| `fusion.rs`       | IMU sensor fusion: complementary/Mahony + optional Madgwick, bias.    |
| `mapping.rs`      | Orientation → mouse delta: deadzone, smoothing, aim curve, v-lock.    |
| `mouse.rs`        | `SendInput` emitter with sub-pixel accumulation + rate coalescing.   |
| `input.rs`        | `GetAsyncKeyState` polling for LMB/keys and hotkey combo parsing.    |
| `tray.rs`         | Native Win32 tray icon + hidden window message loop.                 |
| `sys.rs`          | Platform helpers: timer resolution, thread priority, beeps.          |
| `logger.rs`       | Tiny timestamped logger (stderr / optional file).                    |
| `cli.rs`          | Argument parsing.                                                     |
| `main.rs`         | App state + tracker thread + hotkey thread + orchestration.          |

## Known limitations

- The hotkey/combo keys are **not suppressed**, so they can still reach the game.
- Without a magnetometer, absolute **yaw drifts slowly**; this is irrelevant because only
  *relative* head motion is emitted. Auto bias calibration keeps drift negligible in practice.
- `SendInput` injection may be flagged by aggressive anti-cheat systems. Run as administrator
  and at your own risk in multiplayer titles.

## License

MIT (same as `ar-drivers`).

---

Built and tested via GitHub Actions (`.github/workflows/ci.yml`): `cargo check` → `cargo test` → `cargo build --release` on `x86_64-pc-windows-msvc`.
