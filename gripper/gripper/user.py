#!/usr/bin/env python3

# This node interacts with the user, starts the selected pick sequence,
    # and saves the appropirate metadata. This is the main script for using the gripper.

# ROS imports
import rclpy
from rclpy.node import Node
from gripper_msgs.srv import GripperVacuum
from gripper_msgs.srv import GripperFingers
from gripper_msgs.srv import StartProxy

# standard imports
import numpy as np
import matplotlib.pyplot as plt
import yaml

def validate_user_input(input_str: str, val_str_list):
    """Helper function for validating user input. Checks if a given input from the
        command line is within a given validation list"""
    ret = None
    while ret not in val_str_list:
        ret = input(input_str)

    return ret

class User(Node):
    def __init__(self):
        # initialize and name the node
        super().__init__("user")
        self.get_logger().info("Started user node")

        # set up service clients
        self.proxy_service_client = self.create_client(StartProxy, 'start_proxy_pick')
        self.get_logger().info("Waiting for proxy pick service")
        self.proxy_service_client.wait_for_service()

    def send_proxy_request(self, proxy_status):
        """Function to call gripper vacuum service.
            Inputs - vacuum_status (bool): True turns the vacuum on, False turns it off
        """
        # build the request
        request = StartProxy.Request()
        request.start_proxy = proxy_status
        # make the service call (asynchronously)
        if proxy_status:
            self.get_logger().info("Sending service request to start proxy pick sequence")
        self.vacuum_response = self.proxy_service_client.call_async(request)
    
    def get_user_info(self):
        """Method for grabbing and saving user inputs. Takes input from the 
            command line and saves them to a YAML file
        """
        name = ""
        while name == "":
            name = input("a. Your name: ").lower()

        experiment_type = validate_user_input("b. Experiment type (sim, proxy, real): ",
                                                   ["sim", "proxy", "real"])
        pressure = validate_user_input("c. Feed-in pressure at valve, PSI (60, 65, 70) -- \n"
                             "Tip: if the desired pressure is lower than is currently at the pressure regulator, \n"
                             "then first pass that pressure and then go up to the desired pressure \n"
                             "Pressure: ",
                             ["60", "65", "70"])
        # proxy specific parameters
        if experiment_type == "proxy":
            stiffness = validate_user_input("d. Branch stiffness level (low, medium, high): ", 
                                                 ["low", "medium", "high"])
            magnet_force = validate_user_input("e. Magnet force level (low, medium, high): ",
                                                    ["low", "medium", "high"])
        else:
            stiffness = None
            magnet_force = None

        pick_pattern = validate_user_input("f. Apple pick pattern: (a) Retreat, (b) Rotate and Retreat, (c) Flex and Retreat: ",
                                                ["a", "b", "c"])
        tracks = validate_user_input("g. Gripper tracks (a) v8 70_80, (b) v8 80_90: ",
                                          ["a", "b"])
        actuation_mode = validate_user_input("h. Actuation mode (dual, suction, fingers): ",
                                                  ["dual", "suction", "fingers"])

        # compile and dump into yaml file
        d = {"name":name, "experiment type":experiment_type, "pressure":pressure, "branch stiffness":stiffness, 
             "magnet force":magnet_force, "pick pattern":pick_pattern, "tracks":tracks, "actuation mode":actuation_mode}

        with open ("config/user_parameters.yaml", 'w') as yaml_file:
            yaml.dump(d, yaml_file, default_flow_style=False, sort_keys=False)
    
def main(args=None):
    # initialize rclpy
    rclpy.init(args=args)
    # instantiate the class
    my_user = User()
    my_user.get_user_info()
    my_user.send_proxy_request(True)
    # hand control over to ROS2
    # rclpy.spin(my_user)
    # shutdown cleanly when finished
    # rclpy.shutdown()

if __name__ == "__main__":
    main()