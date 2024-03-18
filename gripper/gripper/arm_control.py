#!/usr/bin/env python3

# This node communicates with the "hello_moveit" node (moveit_service.cpp). I didn't want
# to write any more cpp than I had to, so this adds more functionality that could be in that executable
# - gets the current location of the end effector, formants sends PoseStamped goals to the arm,
# eventually will replan in case of faillure, etc

# ROS imports
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from gripper_msgs.srv import MoveArm, SetArmGoal, GetArmPosition

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

# other imports
import numpy as np

class ArmControl(Node):
    def __init__(self):
        super().__init__("arm_control")

        # set up the services
        self.create_service(SetArmGoal, 'set_arm_goal', self.set_goal_service_callback)
        self.create_service(GetArmPosition, 'get_arm_position', self.get_position_service_callback)

        # set up service clients
        self.arm_service_client = self.create_client(MoveArm, 'move_arm')
        self.get_logger().info("Waiting for arm service")
        self.arm_service_client.wait_for_service()

        # set up some TF stuff
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        

    ##  ------------- SERVICE CALLBACKS ------------- ##
    def set_goal_service_callback(self, request: SetArmGoal.Request, response: SetArmGoal.Response):
        """Callback function for the arm goal service. Uses the list stored
            in request.ee_position to build a pose message and move the arm"""
        self.get_logger().info(f"Moving the arm to {request.goal_ee_location}")
        goal_pose = self.build_pose_stamped(position=request.goal_ee_location,
                                            orientation=request.goal_ee_orientation,
                                            frame=request.frame)
        
        self.send_arm_goal(goal_pose, request.move_cartesian)

        response.result = True
        return response
    
    def get_position_service_callback(self, request: GetArmPosition.Request, response: GetArmPosition.Response):
        """Callback function for the position request service. Sends a PoseStamped
        message of the current end effector position in the desired frame"""

        self.get_logger().info(f"Requesting end effector position in the {request.frame} frame")
        
        try:
            ee_in_new_frame = self.get_current_pose(request.frame)
            response.current_ee_pose = ee_in_new_frame
        except TransformException as e:
            self.get_logger().info(f"Transform failed {e}")
            dummy_frame = PoseStamped()
            response.current_ee_pose = dummy_frame

        return response

    
    ##  ------------- SERVICE CLIENT CALLS ------------- ##

    def send_arm_goal(self, goal_pose:PoseStamped, move_cartesian:bool):
        """Function to call arm service.
            Inputs - 
                goal_pose: PoseStamped message containing the location, orientation, and reference frame
                    of the goal end effector position
        """
        # build the request
        request = MoveArm.Request()
        request.ee_goal = goal_pose
        request.move_cartesian = move_cartesian

        # make the service call (asynchronously)
        self.move_arm_response = self.arm_service_client.call_async(request)

    ##  ------------- HELPER FUNCTIONS ------------- ##
    def build_pose_stamped(self, position, frame, orientation=[0.0,0.0,0.0,1.0]) -> PoseStamped:
        """Method to build a PoseStamped message in a given frame"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        goal_pose.pose.position.x = position[0]
        goal_pose.pose.position.y = position[1]
        goal_pose.pose.position.z = position[2]

        goal_pose.pose.orientation.x = orientation[0]
        goal_pose.pose.orientation.y = orientation[1]
        goal_pose.pose.orientation.z = orientation[2]
        goal_pose.pose.orientation.w = orientation[3]

        return goal_pose
    
    def get_current_pose(self, frame_id):
        """Method that builds a stamped pose for the end effector position and returns
        that pose in the given frame
        
        Inputs - frame_id (string): frame you want the end effector position in
        """
        origin = self.build_pose_stamped(position=[0.0,0.0,0.0], frame="suction_gripper")
        transformed_pose = self.tf_buffer.transform(origin, frame_id, timeout=Duration(seconds=10.0))

        return transformed_pose
    
    def transform_to_world_frame(self, pose_stamped:PoseStamped):
        """Method that converts any given pose stamped message to the world frame"""

        successful_transform = False
        while not successful_transform:
            try:
                new_pose = self.tf_buffer.transform(pose_stamped, "world", Duration(seconds=2.0))
                successful_transform = True
            except TransformException as e:
                self.get_logger().info(f"Transform failed, trying again: {e}")

        return new_pose



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