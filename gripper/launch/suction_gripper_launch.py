import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='gripper',
            executable='suction_gripper'
        ),

        launch_ros.actions.Node(
            package='gripper',
            executable='arm_control'
        ),
    ])