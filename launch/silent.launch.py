from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():

    debug = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("color_blob_centroid"), "launch", "colorblob.launch.py")
        ),
        launch_arguments={
            "continuous_output": "false",
            "show_image": "false",
            "debug": "false",
        }.items(),
    )

    return LaunchDescription([debug])
