# Apple Gripper
### Repository for the ROS2 conversion of Alejo Velasquez's ROS1 framework

**Depends:**

- sudo apt install ros-${ROS_DISTRO}-ur

---

**Running**

To launch just the RVIZ world:

    ros2 launch gripper ur5e_launch.py

To launch the RVIZ world *and* start the nodes for arm control - required before running the pick sequence:

    ros2 launch gripper suction_gripper_launch.py

To run the user interface and start the pick sequence:

    ros2 run gripper user --ros-args --params-file config/apple_proxy_parameters.yaml

---

**Still working on**

- Writing a pyserial wrapper to communicate with the arduino gripper code
- Using tf2 and MoveIt2 to actually move the arm
