#!/usr/bin/env python3

# This node interacts with the user, starts the selected pick sequence,
    # and saves the appropirate metadata. This is the main script for using the gripper.

# ROS imports
import rclpy
from rclpy.node import Node
from gripper_msgs.srv import GripperFingers, GripperVacuum, SetArmGoal, GetArmPosition
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import ParameterDescriptor

from tf_transformations import euler_from_quaternion, quaternion_about_axis, quaternion_from_euler

# standard imports
import numpy as np
import matplotlib.pyplot as plt
import yaml
import os

def validate_user_input(input_str: str, val_str_list) -> str:
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
        self.vacuum_service_client = self.create_client(GripperVacuum, 'set_vacuum_status')
        self.get_logger().info("Waiting for gripper vaccuum server")
        self.vacuum_service_client.wait_for_service()

        self.fingers_service_client = self.create_client(GripperFingers, 'set_fingers_status')
        self.get_logger().info("Waiting for gripper finger server")
        self.fingers_service_client.wait_for_service()

        self.get_pos_service_client = self.create_client(GetArmPosition, 'get_arm_position')
        self.get_logger().info("Waiting for position server")
        self.get_pos_service_client.wait_for_service()

        self.arm_service_client = self.create_client(SetArmGoal, 'set_arm_goal')
        self.get_logger().info("Waiting for arm service")
        self.arm_service_client.wait_for_service()

        # set up publishers and subscribers
        self.apple_markers_publisher = self.create_publisher(Marker, 'apple_markers', 10)
        self.get_logger().info("Started marker publisher")

        # set up parameters, make them dynamic to allow setting from yaml file
        self.declare_parameters(
            namespace = '',
            parameters = [
                ('apple_location', None, ParameterDescriptor(dynamic_typing=True)),
                ('stem_pose', None, ParameterDescriptor(dynamic_typing=True)),
                ('apple_diameter', None, ParameterDescriptor(dynamic_typing=True)),
                ('apple_height', None, ParameterDescriptor(dynamic_typing=True)),
                ('sampling_sphere_ratio', None, ParameterDescriptor(dynamic_typing=True)),
            ],
            )

        # class variables
        self.markers_published = 0

        self.roll_values = {"x": [], "y": [], "z": []}
        self.apple_location = self.get_parameter('apple_location').get_parameter_value().double_array_value
        self.apple_diameter = self.get_parameter('apple_diameter').get_parameter_value().double_value
        sampling_sphere_ratio = self.get_parameter('sampling_sphere_ratio').get_parameter_value().double_value
        self.sampling_sphere_diameter = self.apple_diameter * sampling_sphere_ratio

        self.experiment_type = "proxy"
        self.actuation_mode = "dual"
        self.branch_stiffness = "medium"
        self.magnet_force = "medium"

        suction_cup_give = 0.01 # meters
        self.approach_distance = 2*suction_cup_give + (self.sampling_sphere_diameter - self.apple_diameter)/2
        self.retrieve_distance = -0.130

        self.save_directory = os.path.abspath(os.path.join(os.getcwd(), os.pardir, 'config'))
    
    def proxy_pick_sequence(self):
        """Method that runs through the sequence to use the apple proxy. Called
            if the proxy service recieves True"""
        
        # specify the trial parameters
        roll_points = 3
        self.generate_roll_values(roll_points)
        yaw_values = [0] # degrees
        offset_values = [5/1000] # meters
        num_trials = 1

        # create and publish markers at the apple and sampling sphere location
        self.update_and_publish_apple()

        # Run through all the points to sample: each roll point can have several yaw tests, offsets, and trials
        for roll_point in range(roll_points):
            roll_value = [self.roll_values["x"][roll_point], self.roll_values["y"][roll_point], self.roll_values["z"][roll_point]]
            # set pick distance
            for yaw_value in yaw_values:
                # offset yaw for gripper alignment
                for offset_value in offset_values:
                    for trial in range(num_trials):
                        # record trial
                        self.get_logger().info(f"Trial {trial+1} of {num_trials} at sampled roll location {roll_value}, yaw {yaw_value}, offset {offset_value}")

                        # move to starting position - ARM
                        self.get_logger().info("Moving to initial sample position")
                        starting_orientation, unit_vector = self.find_tangent_orientation(roll_value)
                        roll_marker = self.create_marker_msg(type=2, position=roll_value, scale=[0.01]*3, color=[0.0,0.0,1.0,1.0])
                        self.apple_markers_publisher.publish(roll_marker)
                        self.get_logger().info(f"Sending goal to arm: {roll_value}, {starting_orientation}")
                        self.send_arm_request(ee_location=roll_value, ee_orientation=starting_orientation)
                        
                        # Pause the sequence to ensure the arm moves to the correct spot
                            # Once I get TF working this will be automated
                        move_check = validate_user_input("Did the arm move to the correct location? (yes/no/skip this trial): ",
                                                         ["yes", "no", "skip this trial"])
                        if move_check == "yes":
                            pass
                        elif move_check == "no":
                            print("Replanning")
                            self.send_arm_request(ee_location=roll_value, ee_orientation=starting_orientation)
                        elif move_check == "skip this trial":
                            print("Skipping to the next trial")
                            continue

                        # start rosbag recording
                        # add noise, if using

                        # apply offset_value - ARM

                        # turn on vacuum - GRIPPER
                        self.send_vacuum_request(vacuum_status=True)
                        self.get_logger().info("Gripper vacuum on")

                        # approach apple - ARM
                            # stops early if suction engages - GRIPPER
                        self.get_logger().info("Moving to approach the apple")
                        apple_approach_position = (np.array(roll_value) + self.approach_distance*unit_vector).tolist()
                        approach_marker = self.create_marker_msg(type=2, position=apple_approach_position, scale=[0.01]*3, color=[0.0,1.0,1.0,1.0])
                        self.apple_markers_publisher.publish(approach_marker)
                        self.get_logger().info(f"Sending goal to arm: {apple_approach_position}, {starting_orientation}")
                        self.send_arm_request(ee_location=apple_approach_position, ee_orientation=starting_orientation, move_cartesian=True)
                        
                        # manually label cup engagement
                        self.label_suction_cups()

                        # engage fingers - GRIPPER
                        if self.actuation_mode == "fingers" or self.actuation_mode == "dual":
                            self.send_fingers_request(fingers_status=True)
                            self.get_logger().info("Fingers engaged")

                        # move away from apple - ARM
                        self.get_logger().info("Retrieving the apple")
                        apple_retrieve_position = (np.array(apple_approach_position) + self.retrieve_distance*unit_vector).tolist()
                        retrieve_marker = self.create_marker_msg(type=2, position=apple_retrieve_position, scale=[0.01]*3, color=[0.0,1.0,1.0,1.0])
                        self.apple_markers_publisher.publish(retrieve_marker)
                        self.get_logger().info(f"Sending goal to arm: {apple_retrieve_position}, {starting_orientation}")
                        self.send_arm_request(ee_location=apple_retrieve_position, ee_orientation=starting_orientation, move_cartesian=True)
                    
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

    def send_arm_request(self, ee_location, ee_orientation=[0.0,0.0,0.0,1.0], ee_frame="world", move_cartesian=False):
        """Method to call arm service.
            Inputs - 
                ee_location (float64 array): End effector goal position
                ee_orientation (float64 array): End effector goal orientation. Defaults to whatever
                frame (string): Reference frame of the given location and orientation

        """
        # build the request
        request = SetArmGoal.Request()
        request.goal_ee_location = ee_location
        request.goal_ee_orientation = ee_orientation
        request.frame = ee_frame
        request.move_cartesian = move_cartesian

        # make the service call (asynchronously)
        self.arm_response = self.arm_service_client.call_async(request)

    def get_arm_position(self, frame="world"):
        """Method to get the end effector position in the specified frame
        
        Inputs - frame (string): Desired frame. Defaults to world
        """

        # build the request
        request = GetArmPosition.Request()
        request.frame = frame

        # make the service call
        self.current_arm_position = self.get_pos_service_client.call_async(request).result()
        self.get_logger().info(self.current_arm_position)
    
    ## ------------- HELPER FUNCTIONS  ------------- ##
    def generate_roll_values(self, n_values: int, angle_range_deg=135, plot=False):
        """Method that generates points along a 2D arc. Used to calculate the 
            roll points to sample along the apple surface. Saves the generated sample
            points in the world frame to self.roll_values

            Inputs - n_values (int): number of points to generate
        """
        # set the angular range and step size
        angle_range = np.deg2rad(angle_range_deg)
        angle_step = angle_range / (n_values-1) if n_values > 1 else 1

        # set the size of the sampling sphere
        sphere_rad = self.sampling_sphere_diameter / 2

        # generate n_values points around the sampling sphere and find their coordinates in the world frame
        for i in range(n_values):
            angle = i * angle_step
            angle_deg = np.rad2deg(angle)
            self.get_logger().info(f"Sampling point {i} roll at {np.round(angle_deg,3)} degrees")

            # adjust by self.apple_location to transform into the world frame
            x = 0 + self.apple_location[0]
            y = -sphere_rad * np.sin(angle) + self.apple_location[1]
            z = -sphere_rad * np.cos(angle) + self.apple_location[2]

            self.roll_values["x"].append(x)
            self.roll_values["y"].append(y)
            self.roll_values["z"].append(z)

        if plot:
            ax = plt.figure().add_subplot(projection='3d')
            ax.scatter(self.roll_values["x"], self.roll_values["y"], self.roll_values["z"])
            ax.set_xlabel("X-axis")
            ax.set_ylabel("Y-axis")
            plt.show()

    def find_tangent_orientation(self, current_roll_value):
        """Method that finds the orientation tangent to a sphere at the given
            sampled roll point.\n
            Inputs - current_roll_value (list or array): x,y,z location of sphere\n
            Returns - q (tuple): quaternion representing the angle
                center_unit_vector (np.array): vector from the sampled point to the center of the sphere
            """
        # step 1 - find the vector pointing to the center of the sphere
        delta_x = current_roll_value[0] - self.apple_location[0]
        delta_y = current_roll_value[1] - self.apple_location[1]
        delta_z = current_roll_value[2] - self.apple_location[2]

        center_vector = np.array([-delta_x, -delta_y, -delta_z])
        vector_length = np.linalg.norm(center_vector)
        center_unit_vector = center_vector / vector_length

        # step 2 - make the cross product between that vector and the vector normal to the palm 
            # to obtain the rotation vector
        
        normal_vector = [0, 1, 0]
        rotation_vector = np.cross(normal_vector, center_unit_vector)

        # step 3 - find the angle of rotation via the dot product
        dot = np.dot(normal_vector, center_unit_vector)
        angle = np.arccos(dot)

        # step 4 - find the quaternion from the single axis rotation
        q = quaternion_about_axis(angle, rotation_vector)

        return q, center_unit_vector
    
    def update_and_publish_apple(self):
        """Method that grabs the most recent apple location from the parameter
            and publishes it as a marker"""
        apple_location = self.get_parameter('apple_location').get_parameter_value().double_array_value.tolist()
        self.get_logger().info(f"Apple location: {apple_location}")
        apple_marker = self.create_marker_msg(type=2, position=apple_location, scale=[self.apple_diameter]*3)
        sampling_sphere = self.create_marker_msg(type=2, position=apple_location, scale=[self.sampling_sphere_diameter]*3, color=[0.0,1.0,0.0,0.3])
        self.apple_markers_publisher.publish(apple_marker)
        self.apple_markers_publisher.publish(sampling_sphere)
        self.get_logger().info("Published apple markers")

        return apple_location
        
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
    
    def create_marker_msg(self, type:int, position, scale, frame='world', color=[1.0,0.0,0.0,1.0], text=None) -> Marker:
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
    
    def get_user_info(self):
        """Method for grabbing and saving user inputs. Takes input from the 
            command line and saves them to a YAML file
        """
        name = ""
        while name == "":
            name = input("a. Your name: ").lower()

        experiment_type = validate_user_input("b. Experiment type (sim, proxy, real): ",
                                                   ["sim", "proxy", "real"])
        self.experiment_type = experiment_type
        pressure = validate_user_input("c. Feed-in pressure at valve, PSI (60, 65, 70) -- \n"
                             "Tip: if the desired pressure is lower than is currently at the pressure regulator, \n"
                             "then first pass that pressure and then go up to the desired pressure \n"
                             "Pressure: ",
                             ["60", "65", "70"])
        # proxy specific parameters
        if experiment_type == "proxy":
            stiffness = validate_user_input("d. Branch stiffness level (low, medium, high): ", 
                                                 ["low", "medium", "high"])
            self.branch_stiffness = stiffness
            magnet_force = validate_user_input("e. Magnet force level (low, medium, high): ",
                                                    ["low", "medium", "high"])
            self.magnet_force = magnet_force
        else:
            stiffness = None
            magnet_force = None

        pick_pattern = validate_user_input("f. Apple pick pattern: (a) Retreat, (b) Rotate and Retreat, (c) Flex and Retreat: ",
                                                ["a", "b", "c"])
        tracks = validate_user_input("g. Gripper tracks (a) v8 70_80, (b) v8 80_90: ",
                                          ["a", "b"])
        actuation_mode = validate_user_input("h. Actuation mode (dual, suction, fingers): ",
                                                  ["dual", "suction", "fingers"])
        self.actuation_mode = actuation_mode

        # compile and dump into yaml file
        d = {"name":name, "experiment type":experiment_type, "pressure":pressure, "branch stiffness":stiffness, 
             "magnet force":magnet_force, "pick pattern":pick_pattern, "tracks":tracks, "actuation mode":actuation_mode}

        # with open ("config/user_info.yaml", 'w') as yaml_file:
        #     yaml.dump(d, yaml_file, default_flow_style=False, sort_keys=False)

    def configure_parameter_yaml(self, insert_dict, node_name="user", file_name="params.yaml"):
        """Method for formatting a yaml file that can be read as
            ros params"""
        d = {node_name: {'ros__parameters': {}}}
        d[node_name]['ros__parameters'] = insert_dict
        print(d)

        with open ("config/"+file_name, 'w') as yaml_file:
            yaml.dump(d, yaml_file, default_flow_style=False, sort_keys=False)
        
def main(args=None):
    # initialize rclpy
    rclpy.init(args=args)
    # instantiate the class
    my_user = User()
    my_user.get_user_info()
    input("Hit enter to start proxy test: ")
    my_user.proxy_pick_sequence()

if __name__ == "__main__":
    main()