//! Small platform-specific helpers: thread priority, timer resolution and feedback beeps.
//!
//! On non-Windows targets every function is a no-op so the core of the program can still
//! be compiled and unit-tested on a developer machine.

#[cfg(windows)]
pub mod imp {
    use windows_sys::Win32::Media::Multimedia::{timeBeginPeriod, timeEndPeriod};
    use windows_sys::Win32::System::Threading::{
        GetCurrentThread, SetThreadPriority, THREAD_PRIORITY_HIGHEST, THREAD_PRIORITY_NORMAL,
        THREAD_PRIORITY_TIME_CRITICAL,
    };
    use windows_sys::Win32::UI::WindowsAndMessaging::{MessageBeep, MB_ICONASTERISK, MB_OK};

    /// Raise the resolution of the system timer to 1 ms for the lifetime of the process.
    /// This makes `Sleep(1)` and short waits far more precise, which matters for a
    /// low-latency input loop.
    pub fn raise_timer_resolution() {
        // 0 returns the previous resolution; ignore it. Best-effort.
        unsafe {
            let _ = timeBeginPeriod(1);
        }
    }

    /// Restore the default timer resolution (call at exit if you like; not strictly required).
    #[allow(dead_code)]
    pub fn lower_timer_resolution() {
        unsafe {
            let _ = timeEndPeriod(1);
        }
    }

    /// Set the priority of the calling thread. `realtime` should be used sparingly; it can
    /// starve other system threads. "high" is a good default for the tracker loop.
    pub fn set_thread_priority(priority: ThreadPriority) {
        let p = match priority {
            ThreadPriority::Normal => THREAD_PRIORITY_NORMAL,
            ThreadPriority::Above => THREAD_PRIORITY_HIGHEST,
            ThreadPriority::High => THREAD_PRIORITY_TIME_CRITICAL,
            ThreadPriority::Realtime => THREAD_PRIORITY_TIME_CRITICAL,
        };
        unsafe {
            SetThreadPriority(GetCurrentThread(), p);
        }
    }

    /// Play the system "OK" beep.
    pub fn beep_ok() {
        unsafe {
            let _ = MessageBeep(MB_OK);
        }
    }

    /// Play the system "asterisk" beep (used for a "disabled" cue).
    pub fn beep_off() {
        unsafe {
            let _ = MessageBeep(MB_ICONASTERISK);
        }
    }
}

/// Quiet no-op implementations for non-Windows hosts.
#[cfg(not(windows))]
pub mod imp {
    use crate::config::ThreadPriority;

    /// No-op on non-Windows.
    pub fn raise_timer_resolution() {}
    /// No-op on non-Windows.
    pub fn lower_timer_resolution() {}
    /// No-op on non-Windows.
    pub fn set_thread_priority(_priority: ThreadPriority) {}
    /// No-op on non-Windows.
    pub fn beep_ok() {}
    /// No-op on non-Windows.
    pub fn beep_off() {}
}

pub use imp::*;
