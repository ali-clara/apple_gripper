#!/usr/bin/env python3

# This node runs the temporal gripper pick sequence, interacts with the user, 
    # and saves the appropirate metadata. This is the main script for using the gripper.

# ROS imports
import rclpy
from rclpy.node import Node

# other imports
import numpy as np
import matplotlib.pyplot as plt

class User(Node):
    def __init__(self):
        # initialize and name the node
        super().__init__("user")
        self.get_logger().info("Started user node")

        # class variables - probably most will be set up as params or loaded from a yaml
        self.roll_values = {"x": [], "y": [], "z": []}
        self.apple_diameter = 80 / 1000 # meters

    def proxy_pick_sequence(self):

        roll_points = 3
        roll_values = self.generate_roll_values(roll_points)

        yaw_values = [0, 60] # degrees
        offset_values = [5/1000, 10/1000] # meters
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
                        # approach apple - ARM
                            # stops early if suction engages - GRIPPER
                        # manually label cup engagement
                        # close fingers - GRIPPER
                        # move away from apple - ARM
                        # manually label apple pick
                        # open fingers - GRIPPER
                        # turn off vacuum - GRIPPER
                        # stop rosbag recording
                        # save metadata
                        pass

    def generate_roll_values(self, n_values: int, angle_range_deg=135, plot=False):
        """Function that generates points along a 2D arc. Used to calculate the 
            roll points to sample along the apple surface

            Inputs - n_values (int), number of points to generate
        """

        # set the angular range and step size
        angle_range = np.deg2rad(angle_range_deg)
        angle_step = angle_range / (n_values-1)

        # set the size of the sampling sphere
        sphere_rad = (self.apple_diameter / 2) * 1.5 # PROBABLY THIS IS A PARAMETER

        for i in range(n_values):
            angle = i * angle_step
            self.get_logger().info(f"Roll point {i} at {angle}deg")

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

def main(args=None):
    print("user")

if __name__ == "__main__":
    main()