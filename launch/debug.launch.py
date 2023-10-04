from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
import os


def generate_launch_description():

    debug = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("color_blob_centroid"), 'launch','colorblob.launch.py')),
        launch_arguments={
            "continuous_output": "true",
            "show_image": "true",
            "debug": "true",
        }.items(),
    )

    return LaunchDescription([debug])