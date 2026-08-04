//! System tray icon + a tiny hidden window that owns the message loop.
//!
//! Everything here is raw Win32 (`windows-sys`) to avoid the overhead and latency of a full
//! `winit`/windowing stack. The hidden window exists only so the tray icon can receive its
//! callback message and a 1-second timer tick. On non-Windows hosts the tray is replaced by a
//! simple polling loop so the program still runs (e.g. for tests).

use std::sync::Arc;
use std::time::Duration;

use crate::AppState;

// ---- Local copies of stable Windows message/constant IDs (avoid relying on exact exports) ----
const WM_USER: u32 = 0x0400;
const WM_TRAY: u32 = WM_USER + 1;
const NIM_ADD: u32 = 0;
const NIM_DELETE: u32 = 2;
const NIF_MESSAGE: u32 = 0x1;
const NIF_ICON: u32 = 0x2;
const NIF_TIP: u32 = 0x4;
const NIF_INFO: u32 = 0x10;
const NIIF_INFO: u32 = 0x1;
const WM_RBUTTONUP: u32 = 0x0205;
const WM_LBUTTONUP: u32 = 0x0202;
const WM_DESTROY: u32 = 0x0002;
const WM_CLOSE: u32 = 0x0010;
const WM_TIMER: u32 = 0x0113;
const WM_NULL: u32 = 0x0000;
const TPM_RETURNCMD: u32 = 0x0100;
const TPM_RIGHTBUTTON: u32 = 0x0002;
const TPM_RIGHTALIGN: u32 = 0x0020;
const TPM_BOTTOMALIGN: u32 = 0x0020;
const MF_STRING: u32 = 0x0;
const MF_SEPARATOR: u32 = 0x800;
const MF_CHECKED: u32 = 0x8;
const IDI_APPLICATION: usize = 32512;

const MENU_TOGGLE: u32 = 1;
const MENU_VLOCK: u32 = 2;
const MENU_RECENTER: u32 = 3;
const MENU_RECAL: u32 = 4;
const MENU_OPENCFG: u32 = 5;
const MENU_RELOAD: u32 = 6;
const MENU_EXIT: u32 = 7;

#[cfg(windows)]
mod imp {
    use std::sync::atomic::{AtomicU32, Ordering};
    use std::sync::OnceLock;

    use windows_sys::Win32::Foundation::{HWND, LPARAM, LRESULT, WPARAM};
    use windows_sys::Win32::UI::Shell::{NIM_MODIFY, NOTIFYICONDATAW, Shell_NotifyIconW};
    use windows_sys::Win32::UI::WindowsAndMessaging::{
        AppendMenuW, CreatePopupMenu, CreateWindowExW, DefWindowProcW, DestroyMenu, DispatchMessageW,
        GetCursorPos, GetMessageW, LoadIconW, PostMessageW, PostQuitMessage, RegisterClassW,
        RegisterWindowMessageW, SetForegroundWindow, SetTimer, TrackPopupMenu, TranslateMessage,
        WNDCLASSW, MSG, POINT,
    };

    use super::*;

    static TRAY_STATE: OnceLock<Arc<AppState>> = OnceLock::new();
    static TASKBAR_CREATED: AtomicU32 = AtomicU32::new(0);

    /// Build a null-terminated UTF-16 vector from a `&str`.
    fn wide(s: &str) -> Vec<u16> {
        s.encode_utf16().chain(std::iter::once(0)).collect()
    }

    /// Copy a string into a fixed `sz*` field, truncating to `len-1` chars.
    fn write_tip(field: &mut [u16], s: &str) {
        let w: Vec<u16> = s.encode_utf16().collect();
        let n = w.len().min(field.len() - 1);
        field[..n].copy_from_slice(&w[..n]);
        field[n] = 0;
    }

    unsafe extern "system" fn wnd_proc(hwnd: HWND, msg: u32, wparam: WPARAM, lparam: LPARAM) -> LRESULT {
        let tcm = TASKBAR_CREATED.load(Ordering::Relaxed);
        if tcm != 0 && msg == tcm {
            setup_tray_icon(hwnd);
            return 0;
        }
        match msg {
            WM_TRAY => {
                let ev = (lparam as u32) & 0xFFFF;
                if ev == WM_RBUTTONUP || ev == WM_LBUTTONUP {
                    show_menu(hwnd);
                }
                0
            }
            WM_TIMER => {
                update_tip(hwnd);
                if let Some(s) = TRAY_STATE.get() {
                    if s.quit.load(Ordering::SeqCst) {
                        PostQuitMessage(0);
                    }
                }
                0
            }
            WM_DESTROY => {
                let mut nid: NOTIFYICONDATAW = std::mem::zeroed();
                nid.cbSize = std::mem::size_of::<NOTIFYICONDATAW>() as u32;
                nid.hWnd = hwnd;
                nid.uID = 1;
                Shell_NotifyIconW(NIM_DELETE, &nid);
                PostQuitMessage(0);
                0
            }
            WM_CLOSE => {
                if let Some(s) = TRAY_STATE.get() {
                    s.request_quit();
                }
                PostQuitMessage(0);
                0
            }
            _ => DefWindowProcW(hwnd, msg, wparam, lparam),
        }
    }

    fn setup_tray_icon(hwnd: HWND) {
        let state = match TRAY_STATE.get() {
            Some(s) => s,
            None => return,
        };
        let mut nid: NOTIFYICONDATAW = unsafe { std::mem::zeroed() };
        nid.cbSize = std::mem::size_of::<NOTIFYICONDATAW>() as u32;
        nid.hWnd = hwnd;
        nid.uID = 1;
        nid.uFlags = NIF_MESSAGE | NIF_ICON | NIF_TIP;
        nid.uCallbackMessage = WM_TRAY;
        nid.hIcon = unsafe { LoadIconW(std::ptr::null::<u16>(), IDI_APPLICATION as *const u16) };
        write_tip(&mut nid.szTip, "HeadLook — initializing…");
        unsafe {
            Shell_NotifyIconW(NIM_ADD, &nid);
            if state.config.lock().map(|c| c.tray.notify_on_toggle).unwrap_or(false) {
                nid.uFlags = NIF_INFO;
                write_tip_balloon(&mut nid, "HeadLook started", "Press the hotkey (Ctrl+A+R) to toggle head tracking.");
                Shell_NotifyIconW(NIM_MODIFY, &nid);
            }
        }
        update_tip(hwnd);
    }

    fn write_tip_balloon(nid: &mut NOTIFYICONDATAW, title: &str, body: &str) {
        let w: Vec<u16> = title.encode_utf16().collect();
        let n = w.len().min(nid.szInfoTitle.len() - 1);
        nid.szInfoTitle[..n].copy_from_slice(&w[..n]);
        nid.szInfoTitle[n] = 0;
        let b: Vec<u16> = body.encode_utf16().collect();
        let m = b.len().min(nid.szInfo.len() - 1);
        nid.szInfo[..m].copy_from_slice(&b[..m]);
        nid.szInfo[m] = 0;
        nid.dwInfoFlags = NIIF_INFO;
    }

    fn update_tip(hwnd: HWND) {
        let state = match TRAY_STATE.get() {
            Some(s) => s,
            None => return,
        };
        let connected = state.connected.load(Ordering::SeqCst);
        let enabled = state.tracking_enabled.load(Ordering::SeqCst);
        let paused = state.paused_by_button.load(Ordering::SeqCst);
        let status = if !connected {
            "Glasses disconnected"
        } else if !enabled {
            "Tracking OFF"
        } else if paused {
            "Tracking (paused: button held)"
        } else {
            "Tracking ON"
        };
        let tip = format!("HeadLook — {status}");
        let mut nid: NOTIFYICONDATAW = unsafe { std::mem::zeroed() };
        nid.cbSize = std::mem::size_of::<NOTIFYICONDATAW>() as u32;
        nid.hWnd = hwnd;
        nid.uID = 1;
        nid.uFlags = NIF_TIP;
        write_tip(&mut nid.szTip, &tip);
        unsafe {
            Shell_NotifyIconW(NIM_MODIFY, &nid);
        }
    }

    fn show_menu(hwnd: HWND) {
        let state = match TRAY_STATE.get() {
            Some(s) => s.clone(),
            None => return,
        };
        let menu = unsafe { CreatePopupMenu() };
        if menu == std::ptr::null_mut() {
            return;
        }
        let enabled = state.tracking_enabled.load(Ordering::SeqCst);
        let vlock = state.vertical_lock.load(Ordering::SeqCst);
        unsafe {
            AppendMenuW(menu, if enabled { MF_STRING | MF_CHECKED } else { MF_STRING | MF_UNCHECKED }, MENU_TOGGLE, wide("Toggle head tracking\t(Ctrl+A+R)").as_ptr());
            AppendMenuW(menu, if vlock { MF_STRING | MF_CHECKED } else { MF_STRING | MF_UNCHECKED }, MENU_VLOCK, wide("Vertical lock (yaw only)").as_ptr());
            AppendMenuW(menu, MF_SEPARATOR, 0, std::ptr::null());
            AppendMenuW(menu, MF_STRING, MENU_RECENTER, wide("Recenter view").as_ptr());
            AppendMenuW(menu, MF_STRING, MENU_RECAL, wide("Re-calibrate gyro bias").as_ptr());
            AppendMenuW(menu, MF_STRING, MENU_OPENCFG, wide("Open config…").as_ptr());
            AppendMenuW(menu, MF_STRING, MENU_RELOAD, wide("Reload config").as_ptr());
            AppendMenuW(menu, MF_SEPARATOR, 0, std::ptr::null());
            AppendMenuW(menu, MF_STRING, MENU_EXIT, wide("Exit").as_ptr());

            let mut pt: POINT = std::mem::zeroed();
            GetCursorPos(&mut pt);
            SetForegroundWindow(hwnd);
            let id = TrackPopupMenu(
                menu,
                TPM_RETURNCMD | TPM_RIGHTBUTTON | TPM_RIGHTALIGN | TPM_BOTTOMALIGN,
                pt.x,
                pt.y,
                0,
                hwnd,
                std::ptr::null(),
            );
            DestroyMenu(menu);
            // Per MSDN, post a benign message so the window can regain foreground focus.
            PostMessageW(hwnd, WM_NULL, 0, 0);
            handle_menu(id as u32, &state);
        }
    }

    fn handle_menu(id: u32, state: &AppState) {
        match id {
            MENU_TOGGLE => {
                let was_on = state.tracking_enabled.load(Ordering::SeqCst);
                state.tracking_enabled.store(!was_on, Ordering::SeqCst);
                if let Ok(cfg) = state.config.lock() {
                    if cfg.mouse.sound_feedback {
                        if !was_on {
                            crate::sys::imp::beep_ok();
                        } else {
                            crate::sys::imp::beep_off();
                        }
                    }
                }
            }
            MENU_VLOCK => {
                state.vertical_lock.fetch_xor(true, Ordering::SeqCst);
            }
            MENU_RECENTER => state.recenter_request.store(true, Ordering::SeqCst),
            MENU_RECAL => state.recalibrate_request.store(true, Ordering::SeqCst),
            MENU_OPENCFG => {
                let path = state.config_path.to_string_lossy().to_string();
                if std::process::Command::new("notepad.exe").arg(&path).spawn().is_err() {
                    crate::log_warn!("Could not open {} — open it manually", path);
                }
            }
            MENU_RELOAD => state.config_reload_request.store(true, Ordering::SeqCst),
            MENU_EXIT => {
                state.request_quit();
                unsafe { PostQuitMessage(0); }
            }
            _ => {}
        }
    }

    /// Run the tray + message loop. Returns the process exit code.
    pub fn run(state: Arc<AppState>) -> i32 {
        TRAY_STATE.set(state).ok();
        let class_name = wide("HeadLookTray");
        let wc = WNDCLASSW {
            style: 0,
            lpfnWndProc: Some(wnd_proc),
            cbClsExtra: 0,
            cbWndExtra: 0,
            hInstance: std::ptr::null_mut(),
            hIcon: unsafe { LoadIconW(std::ptr::null::<u16>(), IDI_APPLICATION as *const u16) },
            hCursor: std::ptr::null_mut(),
            hbrBackground: std::ptr::null_mut(),
            lpszMenuName: std::ptr::null(),
            lpszClassName: class_name.as_ptr(),
        };
        unsafe {
            RegisterClassW(&wc);
        }
        let hwnd = unsafe {
            CreateWindowExW(
                0,
                class_name.as_ptr(),
                wide("HeadLook").as_ptr(),
                0,
                0,
                0,
                0,
                0,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null(),
            )
        };
        setup_tray_icon(hwnd);
        let taskbar_msg = unsafe { RegisterWindowMessageW(wide("TaskbarCreated").as_ptr()) };
        if taskbar_msg != 0 {
            TASKBAR_CREATED.store(taskbar_msg, Ordering::Relaxed);
        }
        unsafe {
            SetTimer(hwnd, 1, 1000, None);
        }

        let mut msg: MSG = unsafe { std::mem::zeroed() };
        loop {
            let r = unsafe { GetMessageW(&mut msg, hwnd, 0, 0) };
            if r == 0 {
                break; // WM_QUIT
            }
            if r == -1 {
                break;
            }
            unsafe {
                TranslateMessage(&msg);
                DispatchMessageW(&msg);
            }
            if let Some(s) = TRAY_STATE.get() {
                if s.quit.load(Ordering::SeqCst) {
                    break;
                }
            }
        }
        0
    }

    /// No-op on Windows (the message loop already observes the quit flag).
    pub fn request_quit() {}
}

/// Headless fallback for non-Windows hosts.
#[cfg(not(windows))]
mod imp {
    use std::sync::Arc;

    use crate::AppState;

    /// Run a headless loop until quit is requested.
    pub fn run(state: Arc<AppState>) -> i32 {
        crate::log_info!("Tray disabled on this platform; running headless. Press Ctrl-C to quit.");
        while !state.quit.load(std::sync::atomic::Ordering::SeqCst) {
            std::thread::sleep(Duration::from_millis(200));
        }
        0
    }

    /// No-op on non-Windows hosts.
    pub fn request_quit() {}
}

pub use imp::*;
