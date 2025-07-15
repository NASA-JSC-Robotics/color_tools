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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="wrist_mounted_camera",
            description="prefix of image topic you are listening to.\
                        (i.e. realsense camera_name parameter when launching realsense)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_hardware",
            default_value="false",
            description="Simulate camera information instead of bringing up actual cameras",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "continuous_output",
            default_value="true",
            description="continually output the transform when color blob is visible instead of once per service call",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "show_image",
            default_value="true",
            description="Show an image from the camera - only available if mock_harware is false",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            description="Show the underlying color segmentation from the camera,"
            "Shows world coords and blob size - only available if mock_harware is false",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "color_img_topic",
            default_value="color/image_raw",
            description="Topic (not including prefix) on which raw color image is published.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "depth_img_topic",
            default_value="aligned_depth_to_color/image_raw",
            description="Topic (not including prefix) on which raw depth image is published.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "cam_info_topic",
            default_value="color/camera_info",
            description="Topic (not including prefix) on which camera info is published.",
        )
    )

    prefix = LaunchConfiguration("prefix")
    mock_hw = LaunchConfiguration("mock_hardware")
    show_img = LaunchConfiguration("show_image")
    debug = LaunchConfiguration("debug")
    cont_output = LaunchConfiguration("continuous_output")
    color_img_topic = LaunchConfiguration("color_img_topic")
    depth_img_topic = LaunchConfiguration("depth_img_topic")
    cam_info_topic = LaunchConfiguration("cam_info_topic")

    colorblob_node = Node(
        package="color_blob_centroid",
        executable="ColorBlobCentroid",
        name="ColorBlobCentroid",
        output="screen",
        parameters=[
            {
                "prefix": prefix,
                "mock_hardware": mock_hw,
                "show_image": show_img,
                "debug": debug,
                "continuous_output": cont_output,
                "color_img_topic": color_img_topic,
                "depth_img_topic": depth_img_topic,
                "cam_info_topic": cam_info_topic,
            }
        ],
    )

    nodes_to_launch = [colorblob_node]

    return LaunchDescription(declared_arguments + nodes_to_launch)
