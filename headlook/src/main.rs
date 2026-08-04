//! HeadLook — head-tracked mouse control for XREAL Air / compatible AR glasses on Windows.
//!
//! Reads IMU data from the glasses via `ar-drivers`, fuses it into a head orientation, and
//! translates yaw/pitch into low-latency relative mouse motion emitted with `SendInput`.
//! A background tray icon provides toggles; a global hotkey (Ctrl+A+R) enables/disables
//! tracking; holding the left mouse button (or configured extra keys) pauses tracking so you
//! can aim/shoot without the view drifting.

mod cli;
mod config;
mod fusion;
mod input;
mod logger;
mod mapping;
mod mouse;
mod sys;
mod tray;

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use ar_drivers::{any_glasses, ARGlasses, GlassesEvent};

use crate::config::Config;
use crate::fusion::SensorFusion;
use crate::mapping::{HeadAngles, HeadMapper};
use crate::mouse::MouseEmitter;

/// Shared, thread-safe application state.
pub struct AppState {
    /// Path to the active config file.
    pub config_path: PathBuf,
    /// The current configuration (mirrored here so the tracker only takes a lock on epoch change).
    pub config: Mutex<Config>,
    /// Bumped whenever the config should be re-read.
    pub config_epoch: AtomicU64,
    /// Whether head tracking is currently enabled (hotkey / tray toggled).
    pub tracking_enabled: AtomicBool,
    /// Runtime vertical-lock override (tray toggled).
    pub vertical_lock: AtomicBool,
    /// Whether the glasses are currently connected.
    pub connected: AtomicBool,
    /// Whether tracking is paused because a button/key is held.
    pub paused_by_button: AtomicBool,
    /// When true, motion is computed but not emitted (diagnostics).
    pub dry_run: AtomicBool,
    /// Set to shut the whole program down.
    pub quit: AtomicBool,
    /// Request to re-sync the view reference (no output jump).
    pub recenter_request: AtomicBool,
    /// Request to reset the fusion filter (relearn bias).
    pub recalibrate_request: AtomicBool,
    /// Request to reload the config file from disk.
    pub config_reload_request: AtomicBool,
    /// Last computed yaw in degrees (diagnostics / tray).
    pub diag_yaw_deg: AtomicI32,
    /// Last computed pitch in degrees (diagnostics / tray).
    pub diag_pitch_deg: AtomicI32,
}

impl AppState {
    /// Create the initial state.
    pub fn new(config_path: PathBuf, config: Config, dry_run: bool) -> Self {
        let tracking = config.tracking.enabled_on_start;
        Self {
            config_path,
            config: Mutex::new(config),
            config_epoch: AtomicU64::new(0),
            tracking_enabled: AtomicBool::new(tracking),
            vertical_lock: AtomicBool::new(false),
            connected: AtomicBool::new(false),
            paused_by_button: AtomicBool::new(false),
            dry_run: AtomicBool::new(dry_run),
            quit: AtomicBool::new(false),
            recenter_request: AtomicBool::new(false),
            recalibrate_request: AtomicBool::new(false),
            config_reload_request: AtomicBool::new(false),
            diag_yaw_deg: AtomicI32::new(0),
            diag_pitch_deg: AtomicI32::new(0),
        }
    }

    /// Tell every loop to exit.
    pub fn request_quit(&self) {
        self.quit.store(true, Ordering::SeqCst);
    }

    /// Toggle tracking; returns the new state.
    pub fn toggle_tracking(&self) -> bool {
        !self.tracking_enabled.fetch_xor(true, Ordering::SeqCst)
    }
}

fn main() {
    let args = cli::parse();
    if args.help {
        print!("{}", cli::USAGE);
        return;
    }

    let level = if args.verbose {
        logger::Level::Debug
    } else {
        logger::Level::Info
    };
    logger::init(level, args.log_file.clone());

    crate::log_info!("HeadLook starting (build {})", env!("CARGO_PKG_VERSION"));

    let path = config::config_path(args.config.as_deref());
    let cfg = config::load_or_create(&path, args.verbose);

    let state = Arc::new(AppState::new(path, cfg, args.dry_run));
    // Sync the runtime vertical-lock override to the config default.
    state.vertical_lock.store(
        state.config.lock().unwrap().tracking.vertical_lock,
        Ordering::SeqCst,
    );

    if args.list {
        run_list(&state);
        return;
    }

    // Spawn the background tracker (IMU → mouse).
    let tracker_state = state.clone();
    let tracker = thread::spawn(move || tracker_loop(tracker_state));

    // Spawn the hotkey watcher.
    let hotkey_state = state.clone();
    let hotkey = thread::spawn(move || hotkey_loop(hotkey_state));

    // Run the tray (blocks until the user quits). Without a tray we just poll the quit flag.
    let code = if args.no_tray {
        run_headless(state.clone())
    } else {
        tray::run(state.clone())
    };

    // Signal threads to stop and wait for them.
    state.request_quit();
    tray::request_quit();
    let _ = tracker.join();
    let _ = hotkey.join();
    std::process::exit(code);
}

/// Headless message-loop stand-in (for `--no-tray`): waits until quit is requested.
fn run_headless(state: Arc<AppState>) -> i32 {
    crate::log_info!("Running without a tray. Press Ctrl-C to quit.");
    while !state.quit.load(Ordering::SeqCst) {
        thread::sleep(Duration::from_millis(200));
    }
    0
}

/// Detect glasses and print their info, then exit.
fn run_list(state: &AppState) {
    match any_glasses() {
        Ok(mut g) => {
            crate::log_info!("Found glasses: {}", g.name());
            if let Ok(s) = g.serial() {
                crate::log_info!("Serial: {}", s);
            }
            if let Ok(m) = g.get_display_mode() {
                crate::log_info!("Display mode: {:?}", m);
            }
        }
        Err(e) => {
            crate::log_error!("No glasses detected: {}", e);
        }
    }
}

/// The hotkey watcher thread: polls the configured combo and toggles tracking on the edge.
fn hotkey_loop(state: Arc<AppState>) {
    let mut keys = match input::parse_combo(&state.config.lock().unwrap().tracking.hotkey) {
        Ok(k) => k,
        Err(e) => {
            crate::log_error!("Invalid hotkey: {} — hotkey disabled", e);
            vec![]
        }
    };
    let mut watcher = input::HotkeyWatcher::new(keys);
    let mut last_epoch = state.config_epoch.load(Ordering::SeqCst);

    loop {
        if state.quit.load(Ordering::SeqCst) {
            break;
        }
        let epoch = state.config_epoch.load(Ordering::SeqCst);
        if epoch != last_epoch {
            last_epoch = epoch;
            keys = input::parse_combo(&state.config.lock().unwrap().tracking.hotkey).unwrap_or_default();
            watcher = input::HotkeyWatcher::new(keys);
        }
        if watcher.poll_edge() {
            let now_on = state.toggle_tracking();
            if let Ok(cfg) = state.config.lock() {
                if cfg.mouse.sound_feedback {
                    if now_on {
                        sys::imp::beep_ok();
                    } else {
                        sys::imp::beep_off();
                    }
                }
            }
            crate::log_info!(
                "Head tracking {}",
                if now_on { "ENABLED" } else { "DISABLED" }
            );
        }
        thread::sleep(Duration::from_millis(4));
    }
}

/// Connect to the first supported glasses.
fn connect() -> Result<Box<dyn ARGlasses>, String> {
    match any_glasses() {
        Ok(g) => {
            crate::log_info!("Connected to {}", g.name());
            Ok(g)
        }
        Err(e) => Err(format!("{}", e)),
    }
}

/// Main tracker loop: connect, run a session, reconnect on failure.
fn tracker_loop(state: Arc<AppState>) {
    sys::imp::raise_timer_resolution();
    if let Ok(cfg) = state.config.lock() {
        sys::imp::set_thread_priority(cfg.advanced.thread_priority);
    }

    let mut last_epoch = state.config_epoch.load(Ordering::SeqCst);
    let init_cfg = state.config.lock().unwrap().clone();
    let mut fusion = SensorFusion::new(&init_cfg);
    let mut mapper = HeadMapper::new(&init_cfg, HeadAngles { heading: 0.0, elevation: 0.0, heading_unstable: false });
    let mut emitter = MouseEmitter::new(init_cfg.mouse.emit_rate);
    let mut pause_vks = input::parse_pause_keys(&init_cfg.tracking.pause_keys);

    loop {
        if state.quit.load(Ordering::SeqCst) {
            break;
        }
        let reconnect = state.config.lock().unwrap().advanced.reconnect_seconds.max(0.2);
        match connect() {
            Ok(g) => {
                state.connected.store(true, Ordering::SeqCst);
                let r = run_session(&state, g, &mut fusion, &mut mapper, &mut emitter, &mut last_epoch, &mut pause_vks);
                state.connected.store(false, Ordering::SeqCst);
                match r {
                    Ok(()) => break, // quit requested
                    Err(e) => crate::log_warn!("Session ended: {}", e),
                }
            }
            Err(e) => {
                state.connected.store(false, Ordering::SeqCst);
                crate::log_warn!("Connection failed: {} (retry in {:.1}s)", e, reconnect);
                thread::sleep(Duration::from_secs_f32(reconnect));
            }
        }
    }
    crate::log_info!("Tracker thread exiting");
}

/// One connected session: read IMU, fuse, map, emit.
#[allow(clippy::too_many_arguments)]
fn run_session(
    state: &Arc<AppState>,
    mut glasses: Box<dyn ARGlasses>,
    fusion: &mut SensorFusion,
    mapper: &mut HeadMapper,
    emitter: &mut MouseEmitter,
    last_epoch: &mut u64,
    pause_vks: &mut Vec<i32>,
) -> Result<(), String> {
    let mut last_wall = Instant::now();
    let mut emit_clock = Instant::now();
    let mut last_print = Instant::now();
    let mut timeouts = 0u32;
    let dry = state.dry_run.load(Ordering::SeqCst);
    let timeout_count = state.config.lock().unwrap().advanced.timeout_count.max(1);
    let calibration_seconds = state.config.lock().unwrap().advanced.calibration_seconds.max(0.1);

    loop {
        if state.quit.load(Ordering::SeqCst) {
            return Ok(());
        }

        // --- config reload requested from the tray ---
        if state.config_reload_request.swap(false, Ordering::SeqCst) {
            let p = state.config_path.clone();
            if let Ok(text) = std::fs::read_to_string(&p) {
                if let Ok(mut c) = Config::from_toml(&text) {
                    c.sanitize();
                    *state.config.lock().unwrap() = c;
                    state.config_epoch.fetch_add(1, Ordering::SeqCst);
                    crate::log_info!("Config reloaded from {}", p.display());
                } else {
                    crate::log_error!("Reload failed: config parse error");
                }
            }
        }

        // --- config epoch change → refresh tunables ---
        let epoch = state.config_epoch.load(Ordering::SeqCst);
        if epoch != *last_epoch {
            *last_epoch = epoch;
            let cfg = state.config.lock().unwrap();
            fusion.refresh(&cfg);
            mapper.refresh(&cfg, HeadMapper::angles_of(fusion.orientation()));
            *emitter = MouseEmitter::new(cfg.mouse.emit_rate);
            *pause_vks = input::parse_pause_keys(&cfg.tracking.pause_keys);
            state.vertical_lock.store(cfg.tracking.vertical_lock, Ordering::SeqCst);
        }

        // --- one-shot requests ---
        if state.recenter_request.swap(false, Ordering::SeqCst) {
            mapper.resync(HeadMapper::angles_of(fusion.orientation()));
        }
        if state.recalibrate_request.swap(false, Ordering::SeqCst) {
            fusion.reset();
            crate::log_info!("Gyro bias recalibration requested");
        }
        // Apply the runtime vertical-lock override every frame.
        mapper.set_vertical_lock(state.vertical_lock.load(Ordering::SeqCst));

        match glasses.read_event() {
            Ok(GlassesEvent::AccGyro { accelerometer, gyroscope, .. }) => {
                timeouts = 0;
                let now = Instant::now();
                let dt = last_wall.elapsed().as_secs_f32().clamp(0.0002, 0.05);
                last_wall = now;

                fusion.update(gyroscope, accelerometer, dt);

                if !state.tracking_enabled.load(Ordering::SeqCst) {
                    // Not tracking: keep orientation fresh and re-sync so resume has no jump.
                    mapper.resync(HeadMapper::angles_of(fusion.orientation()));
                    emitter.clear();
                    state.paused_by_button.store(false, Ordering::SeqCst);
                    continue;
                }

                let paused = input::is_paused(pause_vks);
                state.paused_by_button.store(paused, Ordering::SeqCst);
                if paused {
                    mapper.resync(HeadMapper::angles_of(fusion.orientation()));
                    emitter.clear();
                    continue;
                }

                let angles = HeadMapper::angles_of(fusion.orientation());
                let tau = state.config.lock().unwrap().filter.smoothing_tau;
                let (dx, dy) = mapper.step(angles, dt, tau);
                emitter.add(dx, dy);

                let e_now = Instant::now();
                let edt = emit_clock.elapsed().as_secs_f32().max(1e-5);
                emit_clock = e_now;
                let (ox, oy) = emitter.tick(edt);
                if !dry && (ox != 0 || oy != 0) {
                    mouse::emit_relative(ox, oy);
                }

                if last_print.elapsed() >= Duration::from_millis(250) {
                    last_print = Instant::now();
                    let a = HeadMapper::angles_of(fusion.orientation());
                    state.diag_yaw_deg.store((a.heading * 180.0 / std::f32::consts::PI) as i32, Ordering::SeqCst);
                    state.diag_pitch_deg.store((a.elevation * 180.0 / std::f32::consts::PI) as i32, Ordering::SeqCst);
                    if dry {
                        let b = fusion.bias();
                        crate::log_info!(
                            "yaw={:6.1}° pitch={:6.1}° out=({},{}) bias=({:.4},{:.4},{:.4}) still={:.2}s",
                            a.heading.to_degrees(),
                            a.elevation.to_degrees(),
                            ox,
                            oy,
                            b.x,
                            b.y,
                            b.z,
                            fusion.still_time()
                        );
                    }
                }
            }
            Ok(_) => { /* key press / proximity / etc. — ignore for mouse control */ }
            Err(e) => {
                let msg = format!("{}", e);
                let is_timeout = msg.contains("timeout") || msg.contains("Packet");
                if is_timeout {
                    timeouts += 1;
                    if timeouts >= timeout_count {
                        return Err(format!("device unresponsive ({} timeouts): {}", timeouts, e));
                    }
                } else {
                    return Err(format!("glasses error: {}", e));
                }
            }
        }
    }
}
