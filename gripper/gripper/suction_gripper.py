#!/usr/bin/env python3

# This node communicates with the arduino mega via Pyserial to control
    # the gripper - vacuum on/off, fingers open/close, etc

# ROS imports
import rclpy
from rclpy.node import Node
from gripper_msgs.srv import GripperVacuum, GripperFingers

# standard imports
import numpy as np

class SuctionGripper(Node):
    def __init__(self):
        # initialize and name the node
        super().__init__("suction_gripper")

        # set up the services
        self.vacuum_service = self.create_service(GripperVacuum, 'set_vacuum_status', self.vacuum_service_callback)
        self.fingers_service = self.create_service(GripperFingers, 'set_fingers_status', self.fingers_service_callback)

        # class variables
        self.vacuum_on = False
        self.fingers_engaged = False

    def vacuum_service_callback(self, request, response):
        """Callback function for the vacuum service. Uses the bool stored in set_vacuum
            to turn the vacuum on or off
        """
        # if the request is True, turn the vacuum on
        if request.set_vacuum:
            self.vacuum_on = True
            self.get_logger().info("Gripper vacuum on")
        elif not request.set_vacuum:
            self.vacuum_on = False
            self.get_logger().info("Gripper vacuum off")

        response.result = True
        return response
    
    def fingers_service_callback(self, request, response):
        """Callback function for the fingers service. Uses the bool stored in set_fingers
            to engage or disengage the grippers. This might be better as an action.
        """
        # if the request is True, engage the fingers
        if request.set_fingers:
            self.fingers_engaged = True
            self.get_logger().info("Fingers engaged")
        elif not request.set_fingers:
            self.fingers_engaged = False
            self.get_logger().info("Fingers disengaged")

        response.result = True
        return response

def main(args=None):
    # initialize
    rclpy.init(args=args)
    # instantiate the class
    my_gripper = SuctionGripper()
    # hand control over to ROS2
    rclpy.spin(my_gripper)
    # shutdown cleanly
    rclpy.shutdown()

if __name__ == "__main__":
    main()