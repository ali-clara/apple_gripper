#!/usr/bin/env python3

# This node runs the temporal gripper pick sequence for the apple proxy
    # if activated by user.py

# ROS imports
import rclpy
from rclpy.node import Node
from gripper_msgs.srv import GripperVacuum, GripperFingers, StartProxy
from visualization_msgs.msg import Marker, MarkerArray
from rcl_interfaces.msg import ParameterDescriptor

# standard imports
import numpy as np
import matplotlib.pyplot as plt
import yaml
import sys

# other imports
sys.path.append("/home/aliclara/ros2_ws/src/apple_gripper/gripper/gripper")
from user import validate_user_input

class ProxyPick(Node):
    def __init__(self):
        # initialize and name the node
        super().__init__('proxy_pick')

        # set up services
        self.start_proxy_service = self.create_service(StartProxy, 'start_proxy_pick', self.proxy_pick_callback)
        self.get_logger().info("Started proxy pick service")
        
        # set up service clients
        self.vacuum_service_client = self.create_client(GripperVacuum, 'set_vacuum_status')
        self.get_logger().info("Waiting for gripper vaccuum server")
        self.vacuum_service_client.wait_for_service()

        self.fingers_service_client = self.create_client(GripperFingers, 'set_fingers_status')
        self.get_logger().info("Waiting for gripper finger server")
        self.fingers_service_client.wait_for_service()

        # set up publishers and subscribers
        self.apple_markers_publisher = self.create_publisher(Marker, 'apple_markers', 1)
        
        # set up parameters
        self.declare_parameters(
            namespace = '',
            parameters = [
                ('apple_pose', None, ParameterDescriptor(dynamic_typing=True)),
                ('stem_pose', None, ParameterDescriptor(dynamic_typing=True)),
                ('apple_diameter', None, ParameterDescriptor(dynamic_typing=True)),
                ('apple_height', None, ParameterDescriptor(dynamic_typing=True)),
            ],
            )

        # class variables - probably most will be set up as params or loaded from a yaml
        self.roll_values = {"x": [], "y": [], "z": []}
        self.apple_diameter = 80 / 1000 # meters
        self.actuation_mode = "dual"
        self.markers_published = 0
        self.apple_marker_array = []
        # self.apple_pose = [-0.69, -0.275, 1.05, 0.0, 0.0, 0.0]
        self.apple_pose = self.get_parameter('apple_pose').get_parameter_value().double_array_value
        self.sampling_sphere_diameter = self.apple_diameter * 1.5 # PROBABLY THIS IS A PARAMETER

    def proxy_pick_callback(self, request, response):
        """Callback function for the proxy pick service. Should probably 
            add a feature to allow the user to cancel it."""
        if request.start_proxy:
            self.get_logger().info("Starting proxy pick sequence")
            self.proxy_pick_sequence()

        response.result = True
        return response
    
    def proxy_pick_sequence(self):
        """Method that runs through the sequence to use the apple proxy. Called
            if the proxy service recieves True"""

        roll_points = 1
        roll_values = self.generate_roll_values(roll_points)

        yaw_values = [0] # degrees
        offset_values = [5/1000] # meters
        num_trials = 1

        # create and publish markers at the apple and sampling sphere location
        # nab the most updated version of the apple position parameter
        apple_pose = self.get_parameter('apple_pose').get_parameter_value().double_array_value
        apple_marker = self.create_marker_msg(type=2, position=apple_pose, scale=[self.apple_diameter]*3)
        sampling_sphere = self.create_marker_msg(type=2, position=apple_pose, scale=[self.sampling_sphere_diameter]*3, color=[0.0,1.0,0.0,0.3])
        self.apple_markers_publisher.publish(apple_marker)
        self.apple_markers_publisher.publish(sampling_sphere)

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
                        self.label_suction_cups()
                        # engage fingers - GRIPPER
                        if self.actuation_mode == "fingers" or self.actuation_mode == "dual":
                            self.send_fingers_request(fingers_status=True)
                            self.get_logger().info("Fingers engaged")
                        # move away from apple - ARM
                        # manually label apple pick
                        self.label_apple_pick()
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

    ##  ------------- SERVICE CLIENT CALLS ------------- ##
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

    
    ## ------------- HELPER FUNCTIONS  ------------- ##
    def generate_roll_values(self, n_values: int, angle_range_deg=135, plot=False):
        """Function that generates points along a 2D arc. Used to calculate the 
            roll points to sample along the apple surface

            Inputs - n_values (int): number of points to generate
        """
        # set the angular range and step size
        angle_range = np.deg2rad(angle_range_deg)
        angle_step = angle_range / (n_values-1) if n_values > 1 else 1

        # set the size of the sampling sphere
        sphere_rad = self.sampling_sphere_diameter / 2

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

    def label_suction_cups(self):
        """Method to allow the user to label the suction cup engagement after
            the initial approach
        """
        # https://stackoverflow.com/questions/287871/how-do-i-print-colored-text-to-the-terminal
        CRED = '\033[91m'
        CGREEN = '\033[92m'
        CEND = '\033[0m'

        self.get_logger().info("\n ****Label the engagement of the suction cups****")

        cupA_engaged = validate_user_input("Is "+CGREEN+"suction cup A "+CEND+"engaged? (yes, no): ",
                                           ["yes", "no"])
        
        cupB_engaged = validate_user_input("Is "+CGREEN+"suction cup B "+CEND+"engaged? (yes, no): ",
                                           ["yes", "no"])
        
        cupC_engaged = validate_user_input("Is "+CGREEN+"suction cup C "+CEND+"engaged? (yes, no): ",
                                           ["yes", "no"])
        
    def label_apple_pick(self):
        """Method to allow the user to label the apple pick result"""

        self.get_logger().info("\n ****Label the result of the apple pick****")

        pick_result = validate_user_input("(a) Successful: after pick pattern \n"
                                          "(b) Successful: before pick pattern \n"
                                          "(c) Unsuccessful: apple picked but fell \n"
                                          "(d) Unsuccessful: apple not picked \n"
                                          "Result: ",
                                          ["a", "b", "c", "d"])
    
    def create_marker_msg(self, type:int, position, scale, frame='/map', color=[1.0,0.0,0.0,1.0], text=None) -> Marker:
        """Method to create a marker given input parameters"""
        marker = Marker()

        # set the appropriate marker fields
        marker.type = type
        marker.header.frame_id = frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = self.markers_published
        self.markers_published += 1
        # add the marker
        marker.action = 0

        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]

        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.w = 1.0

        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        if text is not None:
            marker.text = text

        return marker

def main(args=None):
    # initialize rclpy
    rclpy.init(args=args)
    # instantiate the class + node
    my_proxy_pick = ProxyPick()
    # hand control over to ROS2
    rclpy.spin(my_proxy_pick)
    # shutdown cleanly
    rclpy.shutdown()

if __name__ == "__main__":
    main()

