#!/usr/bin/env python3

# This node runs the temporal gripper pick sequence, interacts with the user, 
    # and saves the appropirate metadata. This is the main script for using the gripper.

# ROS imports
import rclpy
from rclpy.node import Node
from gripper_msgs.srv import GripperVacuum
from gripper_msgs.srv import GripperFingers

# other imports
import numpy as np
import matplotlib.pyplot as plt

class User(Node):
    def __init__(self):
        # initialize and name the node
        super().__init__("user")
        self.get_logger().info("Started user node")

        # set up service clients
        self.vacuum_service_client = self.create_client(GripperVacuum, 'set_vacuum_status')
        self.get_logger().info("Waiting for gripper vaccuum server")
        self.vacuum_service_client.wait_for_service()

        self.fingers_service_client = self.create_client(GripperFingers, 'set_fingers_status')
        self.get_logger().info("Waiting for gripper finger server")
        self.fingers_service_client.wait_for_service()

        # class variables - probably most will be set up as params or loaded from a yaml
        self.roll_values = {"x": [], "y": [], "z": []}
        self.apple_diameter = 80 / 1000 # meters
        self.actuation_mode = "dual"
        self.experiment_type = "proxy"

    def proxy_pick_sequence(self):

        roll_points = 1
        roll_values = self.generate_roll_values(roll_points)

        yaw_values = [0] # degrees
        offset_values = [5/1000] # meters
        num_trials = 1

        for roll_point in range(roll_points):
            # set pick distance
            for yaw_value in yaw_values:
                # offset yaw for gripper alignment
                for offset_value in offset_values:
                    for trial in range(num_trials):
                        # record trial
                        # move to starting position - ARM
                        # start rosbag recording
                        # add noise, if using
                        # apply offset_value - ARM
                        # turn on vacuum - GRIPPER
                        self.send_vacuum_request(vacuum_status=True)
                        self.get_logger().info("Gripper vacuum on")
                        # approach apple - ARM
                            # stops early if suction engages - GRIPPER
                        # manually label cup engagement
                        # engage fingers - GRIPPER
                        if self.actuation_mode == "fingers" or self.actuation_mode == "dual":
                            self.send_fingers_request(fingers_status=True)
                            self.get_logger().info("Fingers engaged")
                        # move away from apple - ARM
                        # manually label apple pick
                        # disengage fingers - GRIPPER
                        if self.actuation_mode == "fingers" or self.actuation_mode == "dual":
                            self.send_fingers_request(fingers_status=False)
                            self.get_logger().info("Fingers disengaged")
                        # turn off vacuum - GRIPPER
                        self.send_vacuum_request(vacuum_status=False)
                        self.get_logger().info("Gripper vacuum off")
                        # stop rosbag recording
                        # save metadata
                        pass

    def send_vacuum_request(self, vacuum_status):
        """Function to call gripper vacuum service.
            Inputs - vacuum_status (bool): True turns the vacuum on, False turns it off
        """
        # build the request
        request = GripperVacuum.Request()
        request.set_vacuum = vacuum_status
        # make the service call (asynchronously)
        self.vacuum_response = self.vacuum_service_client.call_async(request)

    def send_fingers_request(self, fingers_status):
        """Function to call gripper fingers service.
            Inputs - fingers_status (bool): True engages the fingers, False disengages
        """
        # build the request
        request = GripperFingers.Request()
        request.set_fingers = fingers_status
        # make the service call (asynchronously)
        self.fingers_response = self.fingers_service_client.call_async(request)

    def generate_roll_values(self, n_values: int, angle_range_deg=135, plot=False):
        """Function that generates points along a 2D arc. Used to calculate the 
            roll points to sample along the apple surface

            Inputs - n_values (int): number of points to generate
        """
        # set the angular range and step size
        angle_range = np.deg2rad(angle_range_deg)
        angle_step = angle_range / (n_values-1) if n_values > 1 else 1

        # set the size of the sampling sphere
        sphere_rad = (self.apple_diameter / 2) * 1.5 # PROBABLY THIS IS A PARAMETER

        for i in range(n_values):
            angle = i * angle_step
            angle_deg = np.rad2deg(angle)
            self.get_logger().info(f"Roll point {i} at {np.round(angle_deg,3)} degrees")

            x = 0
            y = -sphere_rad * np.sin(angle)
            z = -sphere_rad * np.cos(angle)

            self.roll_values["x"].append(x)
            self.roll_values["y"].append(y)
            self.roll_values["z"].append(z)

        if plot:
            ax = plt.figure().add_subplot(projection='3d')
            ax.scatter(self.roll_values["x"], self.roll_values["y"], self.roll_values["z"])
            ax.set_xlabel("X-axis")
            ax.set_ylabel("Y-axis")
            plt.show()

    def validate_user_input(self, input_str: str, val_str_list):
        """Method for validating user input"""
        ret = None
        while ret not in val_str_list:
            ret = input(input_str)

        return ret
    
    def get_user_info(self):
        """Method for grabbing and saving user inputs. Takes input from the 
            command line and saves them to a YAML file (or, will soon)
        """
        name = ""
        while name == "":
            name = input("a. Your name: ").lower()

        experiment_type = self.validate_user_input("b. Experiment type (sim, proxy, real): ",
                                                   ["sim", "proxy", "real"])
        pressure = self.validate_user_input("c. Feed-in pressure at valve, PSI (60, 65, 70) -- \n"
                             "Tip: if the desired pressure is lower than is currently at the pressure regulator, \n"
                             "then first pass that pressure and then go up to the desired pressure \n"
                             "Pressure: ",
                             ["60", "65", "70"])
        # proxy specific parameters
        if self.experiment_type == "proxy":
            stiffness = self.validate_user_input("d. Branch stiffness level (low, medium, high): ", 
                                                 ["low", "medium", "high"])
            magnet_force = self.validate_user_input("e. Magnet force level (low, medium, high): ",
                                                    ["low", "medium", "high"])
        pick_pattern = self.validate_user_input("f. Apple pick pattern: (a) Retreat, (b) Rotate and Retreat, (c) Flex and Retreat: ",
                                                ["a", "b", "c"])
        tracks = self.validate_user_input("g. Gripper tracks (a) v8 70_80, (b) v8 80_90: ",
                                          ["a", "b"])
        actuation_mode = self.validate_user_input("h. Actuation mode (dual, suction, fingers): ",
                                                  ["dual", "suction", "fingers"])

def main(args=None):
    # initialize rclpy
    rclpy.init(args=args)
    # instantiate the class
    my_user = User()
    my_user.get_user_info()
    my_user.proxy_pick_sequence()
    # hand control over to ROS2
    # rclpy.spin(my_user)
    # shutdown cleanly when finished
    # rclpy.shutdown()

if __name__ == "__main__":
    main()