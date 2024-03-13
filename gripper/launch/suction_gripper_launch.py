from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    ld = LaunchDescription()
    
    # ld.add_action(IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('ur_robot_driver'),
    #             'launch',
    #             'ur_control.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         "ur_type": "ur5e",
    #         "robot_ip": "yyy.yyy.yyy.yyy",
    #         "use_fake_hardware": "true",
    #         "launch_rviz": "false",
    #     }.items()
    # ))
    
    # Launch the ur5e moveit2 launch file 
    # (also launches rviz and the moveit_service.cpp moveit node)
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gripper'),
                'launch',
                'ur_moveit_launch.py'
            ])
        ]),
    ))
    
    # Launch the node to control gripper functionality
    ld.add_action(Node(
        package='gripper',
        executable='suction_gripper.py',
    ))

    # Launch the node for the intermediary arm control interface
    # (since I didn't want to write more cpp than I needed to)
    ld.add_action(Node(
        package='gripper',
        executable='arm_control.py',
    ))
    
    return ld