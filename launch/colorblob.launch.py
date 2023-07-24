from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
import os


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
            default_value='false',
            description="Simulate camera information instead of bringing up actual cameras",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "continuous_output",
            default_value='false',
            description="continually output the transform when color blob is visible instead of once per service call",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "show_image",
            default_value='false',
            description="Show an image from the camera - only available if mock_harware is false",
        )
    )

    prefix = LaunchConfiguration("prefix")
    mock_hw = LaunchConfiguration("mock_hardware")
    show_img = LaunchConfiguration("show_image")
    cont_output = LaunchConfiguration("continuous_output")

    colorblob_node = Node(
        package="color_blob_centroid",
        executable="ColorBlobCentroid",
        name="ColorBlobCentroid",
        output="screen",
        parameters=[
            {
                "prefix":prefix,
                "mock_hardware":mock_hw,
                "show_image":show_img,
                "continuous_output":cont_output,
            }
        ],
    )

    nodes_to_launch = [
        colorblob_node
    ]

    return LaunchDescription(declared_arguments + nodes_to_launch)
