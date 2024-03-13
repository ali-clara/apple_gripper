# Apple Gripper
### Repository for the ROS2 conversion of Alejo Velasquez's ROS1 framework

**Depends:**

- sudo apt install ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui
- sudo apt install ros-${ROS_DISTRO}-ur

---

**Running**

Launch the UR robot driver

    ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=false

Launch the RVIZ world, start MoveIt2 arm control, and connect to the gripper: - required before running the pick sequence:

    ros2 launch gripper suction_gripper_launch.py

Run the user interface and start the pick sequence:

    ros2 run gripper user --ros-args --params-file config/apple_proxy_parameters.yaml


---

**Still working on**

- Writing a pyserial wrapper to communicate with the arduino gripper code
