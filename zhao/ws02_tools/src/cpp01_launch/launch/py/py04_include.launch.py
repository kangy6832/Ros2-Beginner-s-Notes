from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    include_launch=IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(
                get_package_share_directory(
                    'cpp01_launch'
                ),
                'launch/py',
                'py03_args.launch.py' 
            )
        ),
        launch_arguments={
            "backaround_r":"200",
            "backaround_g":"100",
            "backaround_b":"70",
        }.items()
    )
    return LaunchDescription([include_launch])