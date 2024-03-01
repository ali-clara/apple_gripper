from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    # Launch the ur5e launch file with the given argument
    # (also launches rviz)
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gripper'),
                'launch',
                'ur5e_launch.py'
                ])
            ]),
        launch_arguments = {'ur_type': 'ur5e'}.items()
        ))
    
    ld.add_action(Node(
        package='gripper',
        executable='suction_gripper',
    ))

    ld.add_action(Node(
        package='gripper',
        executable='arm_control',
    ))

    return ld