from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable

def generate_launch_description():
    ld = LaunchDescription()

    joint_limit_params = PathJoinSubstitution([FindPackageShare("gripper"), "config", "ur5e", "joint_limits.yaml"])
    kinematics_params = PathJoinSubstitution([FindPackageShare("gripper"), "config", "ur5e", "default_kinematics.yaml"])
    physical_params = PathJoinSubstitution([FindPackageShare("gripper"), "config", "ur5e", "physical_parameters.yaml"])
    visual_params = PathJoinSubstitution([FindPackageShare("gripper"), "config", "ur5e", "visual_parameters.yaml"])

    robot_description_content = Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("gripper"), "urdf", "scene_description.urdf.xacro"]),
            " ",
            "robot_ip:=172.17.0.2",
            " ",
            "joint_limit_params:=", joint_limit_params,
            " ",
            "kinematics_params:=", kinematics_params,
            " ",
            "physical_params:=", physical_params,
            " ",
            "visual_params:=", visual_params,
            " ",
           "safety_limits:=", "true",
            " ",
            "safety_pos_margin:=", "0.15",
            " ",
            "safety_k_position:=", "20",
            " ",
            "name:=", "ur",
            " ",
            "ur_type:=", "ur3e",
            " ",
            "prefix:=",'""',
            " ",
        ])

    robot_description_semantic_content = Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
            " ",
            "name:=","ur",
            " ",
            "prefix:=",'""',
            " ",
        ])
    
    robot_description = {"robot_description": robot_description_content}
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}
    
    
    # Launch the ur5e launch file with the given argument
    # (also launches rviz)
    # ld.add_action(IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('gripper'),
    #             'launch',
    #             'ur5e_launch.py'
    #             ])
    #         ]),
    #     launch_arguments = {'ur_type': 'ur5e'}.items()
    #     ))
    
    
    # Launch the ur5e moveit2 launch file with the given arguments
    # (also launches rviz)
    # TODO - figure out how to launch my custom rviz file from here
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
    
    # my nodes
    ld.add_action(Node(
        package='gripper',
        executable='suction_gripper.py',
    ))

    ld.add_action(Node(
        package='gripper',
        executable='arm_control.py',
    ))

    ld.add_action(Node(
        package="gripper",
        executable="moveit_service",
        name="moveit_service",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    ))

    return ld