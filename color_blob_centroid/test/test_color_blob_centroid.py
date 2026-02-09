#!/usr/bin/env python3
#
# Copyright (c) 2025, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# This software is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

import numpy as np
import cv2
from sensor_msgs.msg import CameraInfo
from color_tools_msgs.msg import BlobRequest, BlobResult
from cv_bridge import CvBridge
from color_blob_centroid import bindings


def test_detects_red_blob():
    # Configure a dummy test camera
    camera_info = CameraInfo()
    camera_info.header.frame_id = "test_camera"
    camera_info.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
    camera_info.height = 480
    camera_info.width = 640

    # Generate a black image with a 50 pixel red circle in the middle
    color_image_cv = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.circle(color_image_cv, (320, 240), 50, (0, 0, 255), -1)
    depth_image_cv = np.full((480, 640), 0.5, dtype=np.float32)

    # Convert OpenCV images to ROS messages
    bridge = CvBridge()
    color_image_msg = bridge.cv2_to_imgmsg(color_image_cv, encoding="bgr8")
    depth_image_msg = bridge.cv2_to_imgmsg(depth_image_cv, encoding="32FC1")

    # Send and process request for a red blob
    request = BlobRequest()
    request.color = "red"
    request.min_blob_size = 10.0
    request.desired_blob = 0

    result = bindings.process_blobs(
        color_image_msg,
        depth_image_msg,
        camera_info,
        request,
        BlobResult,
    )

    # Verify blob was found
    assert result.centroid_pose.header.frame_id != "", "Frame ID should not be empty"
    assert (
        abs(result.centroid_pose.pose.position.z - 0.5) < 0.01
    ), f"Expected z position ~0.5, got {result.centroid_pose.pose.position.z}"


if __name__ == "__main__":
    test_detects_red_blob()
