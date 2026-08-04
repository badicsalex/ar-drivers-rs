//! Command-line argument parsing (minimal, dependency-free).

use std::path::PathBuf;

/// Parsed command-line arguments.
#[derive(Debug, Clone, Default)]
pub struct Args {
    /// Explicit config path (`--config <path>`).
    pub config: Option<PathBuf>,
    /// Just detect the glasses, print info and exit.
    pub list: bool,
    /// Do not show a system tray; run as a console program.
    pub no_tray: bool,
    /// Compute mouse motion but do not actually emit input (diagnostics).
    pub dry_run: bool,
    /// Verbose logging.
    pub verbose: bool,
    /// Optional log file (useful with `--no-console`).
    pub log_file: Option<PathBuf>,
    /// Print help and exit.
    pub help: bool,
}

/// Short usage string.
pub const USAGE: &str = "\
HeadLook — head-tracked mouse control for XREAL Air / compatible AR glasses.

USAGE:
    headlook [OPTIONS]

OPTIONS:
    -h, --help            Show this help
    -c, --config <PATH>  Use a specific config file
        --list           Detect glasses, print info and exit
        --no-tray        Run without a system tray (console mode)
    -n, --dry-run        Compute motion but do not emit mouse input
    -v, --verbose        Verbose logging
        --log-file <PATH> Write logs to a file (in addition to stderr)
";

/// Parse `std::env::args()`. Unknown flags are ignored (with a warning) to stay robust.
pub fn parse() -> Args {
    let mut args = Args::default();
    let mut iter = std::env::args().skip(1);
    while let Some(a) = iter.next() {
        match a.as_str() {
            "-h" | "--help" => args.help = true,
            "--list" => args.list = true,
            "--no-tray" => args.no_tray = true,
            "-n" | "--dry-run" => args.dry_run = true,
            "-v" | "--verbose" => args.verbose = true,
            "-c" | "--config" => {
                if let Some(p) = iter.next() {
                    args.config = Some(PathBuf::from(p));
                }
            }
            "--log-file" => {
                if let Some(p) = iter.next() {
                    args.log_file = Some(PathBuf::from(p));
                }
            }
            other => {
                // Support `--config=path` style.
                if let Some(rest) = other.strip_prefix("--config=") {
                    args.config = Some(PathBuf::from(rest));
                } else if let Some(rest) = other.strip_prefix("--log-file=") {
                    args.log_file = Some(PathBuf::from(rest));
                } else if other.starts_with('-') {
                    eprintln!("Warning: ignoring unknown argument '{other}'");
                }
            }
        }
    }
    args
}
