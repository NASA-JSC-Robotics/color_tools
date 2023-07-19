from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
import os


def generate_launch_description():
    nodes_to_launch = []
    colorblob_node = Node(
        package="color_blob_centroid",
        executable="ColorBlobCentroid",
        name="ColorBlobCentroid",
        output="screen",
        parameters=[
            {
                "prefix":"wrist_mounted_camera", #prefix of whatever realsense image topic you are listening to (i.e. realsense camera_name parameter when launching realsense)
            }
        ],
    )
    nodes_to_launch.append(colorblob_node)

    return LaunchDescription(nodes_to_launch)
