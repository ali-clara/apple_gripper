#!/usr/bin/env python3

# This node communicates with the arduino mega via Pyserial to control
    # the gripper - vacuum on/off, fingers open/close, etc

# ROS imports
import rclpy
from rclpy.node import Node
from gripper_msgs.srv import GripperVacuum, GripperFingers

# standard imports
import numpy as np
import serial
from serial import SerialException

class SuctionGripper(Node):
    def __init__(self):
        # initialize and name the node
        super().__init__("suction_gripper")

        # set up the services
        self.vacuum_service = self.create_service(GripperVacuum, 'set_vacuum_status', self.vacuum_service_callback)
        self.fingers_service = self.create_service(GripperFingers, 'set_fingers_status', self.fingers_service_callback)

        # set up Pyserial
        arduino_port = "/dev/ttyACM0"
        baud = 115200
        try:
            self.my_serial = serial.Serial(arduino_port, baud)
            self.get_logger().info(f"Connected to serial port {arduino_port}")
            self.real_arduino_flag = True
        except SerialException:
            self.get_logger().info(f"Could not connect to serial port {arduino_port}. Using fake arduino hardware.")
            self.real_arduino_flag = False
            
        # class variables
        self.start_character = "<"
        self.end_character = ">"
        self.vacuum_on_command = "1"
        self.vacuum_off_command = "2"
        self.fingers_engaged_command = "3"
        self.fingers_disengaged_command = "4"

    def read_serial(self):
        get_data = self.my_serial.readline()
        data_string = get_data.decode('utf-8')
        data_raw = data_string[0:][:-2]

        self.get_logger().info(data_raw)

    def vacuum_service_callback(self, request, response):
        """Callback function for the vacuum service. Uses the bool stored in set_vacuum
            to turn the vacuum on or off
        """
        # if the request is True, turn the vacuum on
        if request.set_vacuum:
            # if we're actually connected to an arduino, send the serial message. Otherwise just pretend
            self.get_logger().info("Sending request: gripper vacuum on")
            if self.real_arduino_flag:
                msg_str = self.start_character+self.vacuum_on_command+self.end_character
                self.my_serial.write(str(msg_str).encode())
                self.read_serial()
        # if the request is False, turn the vacuum off
        elif not request.set_vacuum:
            self.get_logger().info("Sending request: gripper vacuum off")
            if self.real_arduino_flag:
                msg_str = self.start_character+self.vacuum_off_command+self.end_character
                self.my_serial.write(str(msg_str).encode())
                self.read_serial()

        response.result = True
        return response
    
    def fingers_service_callback(self, request, response):
        """Callback function for the fingers service. Uses the bool stored in set_fingers
            to engage or disengage the grippers. This might be better as an action.
        """
        # if the request is True, engage the fingers
        if request.set_fingers:
            # like above, if we're actually connected to an arduino, send the serial message. Otherwise just pretend
            self.get_logger().info("Sending request: engage fingers")
            if self.real_arduino_flag:
                msg_str = self.start_character+self.fingers_engaged_command+self.end_character
                self.my_serial.write(str(msg_str).encode())
                self.read_serial()
        # if the request is False, disengage the fingers
        elif not request.set_fingers:
            self.get_logger().info("Sending request: disengage fingers")
            if self.real_arduino_flag:
                msg_str = self.start_character+self.fingers_engaged_command+self.end_character
                self.my_serial.write(str(msg_str).encode())
                self.read_serial()

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