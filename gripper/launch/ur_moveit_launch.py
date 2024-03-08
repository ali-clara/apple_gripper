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
                FindPackageShare('ur_moveit_config'),
                'launch',
                'ur_moveit.launch.py'
                ])
            ]),
        launch_arguments = {'ur_type': 'ur5e',
                            'description_package': 'gripper',
                            'description_file': 'scene_description.urdf.xacro'}.items()
        ))
    
    return ld