import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    ur5e_urdf = os.path.join(get_package_share_directory("ur_description"), "")

    moveit_config = (
        MoveItConfigsBuilder(
            robot_description="ur5e", package_name="gripper"
        )
        .robot_description()
    )