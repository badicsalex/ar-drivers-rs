// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

use std::{sync::Arc, time::Duration};

use ar_drivers::{
    nreal_light::{NrealLight, NrealLightSlamCamera},
    ARGlasses, CameraDescriptor,
};
use opencv::{
    calib3d::init_undistort_rectify_map,
    core::{BORDER_CONSTANT, CV_32FC1, CV_8UC1},
    highgui::{imshow, poll_key},
    imgproc::INTER_LINEAR,
    prelude::Mat,
};

fn undistort_map(camera_desc: &CameraDescriptor) -> (Mat, Mat) {
    let camera_matrix = opencv::core::Mat::from_slice_rows_cols(
        camera_desc.intrinsic_matrix.transpose().as_slice(),
        3,
        3,
    )
    .unwrap();
    let dist_coeffs = opencv::core::Vector::from_slice(camera_desc.distortion.as_slice());
    let rotation_mtx = opencv::core::Mat::from_slice_rows_cols(
        camera_desc
            .stereo_rotation
            .to_rotation_matrix()
            .matrix()
            .as_slice(),
        3,
        3,
    )
    .unwrap();
    let new_camera_matrix = opencv::core::Mat::from_slice_2d(&[
        [250.0, 0.0, 320.0],
        [0.0, 250.0, 240.0],
        [0.0, 0.0, 1.0],
    ])
    .unwrap();
    let mut result1 = Mat::default();
    let mut result2 = Mat::default();
    init_undistort_rectify_map(
        &camera_matrix,
        &dist_coeffs,
        &rotation_mtx,
        &new_camera_matrix,
        opencv::core::Size {
            width: 640,
            height: 480,
        },
        CV_32FC1,
        &mut result1,
        &mut result2,
    )
    .unwrap();
    (result1, result2)
}

fn main() {
    let mut glasses = NrealLight::new().unwrap();
    let camera_descs = glasses.cameras().unwrap();
    let undistort_left = undistort_map(
        camera_descs
            .iter()
            .find(|c| c.name == NrealLight::LEFT_SLAM_CAM)
            .unwrap(),
    );
    let undistort_right = undistort_map(
        camera_descs
            .iter()
            .find(|c| c.name == NrealLight::RIGHT_SLAM_CAM)
            .unwrap(),
    );

    let still_here = Arc::new(());
    let _still_here_clone = still_here.clone();
    std::thread::spawn(move || {
        while Arc::strong_count(&still_here) > 1 {
            // Pump it.
            glasses.read_event().unwrap();
        }
    });

    let mut camera = NrealLightSlamCamera::new().unwrap();

    loop {
        let frame = camera.get_frame(Duration::from_secs(5)).unwrap();
        println!("Got frame with ts {}", frame.timestamp);
        let mat_left = Mat::from_slice_rows_cols(&frame.left, 480, 640).unwrap();
        let mut mat_left_ud =
            Mat::new_rows_cols_with_default(480, 640, CV_8UC1, Default::default()).unwrap();
        opencv::imgproc::remap(
            &mat_left,
            &mut mat_left_ud,
            &undistort_left.0,
            &undistort_left.1,
            INTER_LINEAR,
            BORDER_CONSTANT,
            Default::default(),
        )
        .unwrap();

        let mat_right = Mat::from_slice_rows_cols(&frame.right, 480, 640).unwrap();
        let mut mat_right_ud =
            Mat::new_rows_cols_with_default(480, 640, CV_8UC1, Default::default()).unwrap();
        opencv::imgproc::remap(
            &mat_right,
            &mut mat_right_ud,
            &undistort_right.0,
            &undistort_right.1,
            INTER_LINEAR,
            BORDER_CONSTANT,
            Default::default(),
        )
        .unwrap();
        imshow("Left", &mat_left_ud).unwrap();
        imshow("Right", &mat_right_ud).unwrap();
        let key = poll_key().unwrap();
        if key == 'q' as i32 {
            return;
        }
    }
}
