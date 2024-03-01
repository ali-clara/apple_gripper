#!/usr/bin/env python3

# This node communicates with MoveIt 2 to move the UR5

# ROS imports
import rclpy
from rclpy.node import Node
from gripper_msgs.srv import MoveArm

# MoveIt imports
try:
    from moveit.core.robot_state import RobotState
    from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
    )
except:
    print("Could not find MoveIt2")

# other imports
import numpy as np

class ArmControl(Node):
    def __init__(self):
        super().__init__("arm_control")

        # set up the services
        self.create_service(MoveArm, 'move_arm_goal', self.arm_service_callback)

    def arm_service_callback(self, request, response):
        """Callback function for the arm moving service. Uses the list stored
            in request.ee_position to move the arm"""
        self.get_logger().info(f"Moving the arm to {request.ee_position}")

        response.result = True
        return response

def main(args=None):
    # initialize
    rclpy.init(args=args)
    # instantiate 
    my_arm = ArmControl()
    # hand control over to ROS2
    rclpy.spin(my_arm)
    # shutdown cleanly
    rclpy.shutdown()
    
if __name__ == "__main__":
    main() 