// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

use rusb::{Device, DeviceHandle, DeviceList, GlobalContext};

use crate::{Error, Result};

fn get_device_vid_pid(vid: u16, pid: u16) -> Result<Device<GlobalContext>> {
    for device in DeviceList::new()?.iter() {
        if let Ok(desc) = device.device_descriptor() {
            if desc.vendor_id() == vid && desc.product_id() == pid {
                return Ok(device);
            }
        }
    }
    Err(Error::NotFound)
}

fn get_interface_for_endpoint(device: &Device<GlobalContext>, endpoint_address: u8) -> Option<u8> {
    let config_desc = device.config_descriptor(0).ok()?;
    for interface in config_desc.interfaces() {
        for desc in interface.descriptors() {
            for endpoint in desc.endpoint_descriptors() {
                if endpoint.address() == endpoint_address {
                    return Some(interface.number());
                }
            }
        }
    }
    None
}

pub fn open_device_vid_pid_endpoint(
    vid: u16,
    pid: u16,
    endpoint_address: u8,
) -> Result<DeviceHandle<GlobalContext>> {
    let device = get_device_vid_pid(vid, pid)?;
    let mut device_handle = device.open()?;
    device_handle.set_auto_detach_kernel_driver(true)?;
    device_handle.claim_interface(
        get_interface_for_endpoint(&device, endpoint_address).ok_or_else(|| {
            Error::Other("Could not find endpoint, wrong USB structure (probably)")
        })?,
    )?;
    Ok(device_handle)
}
