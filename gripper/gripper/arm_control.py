#!/usr/bin/env python3

# This node communicates with the "hello_moveit" node (moveit_service.cpp). I didn't want
# to write any more cpp than I had to, so this adds more functionality that could be in that executable
# - gets the current location of the end effector, formants sends PoseStamped goals to the arm,
# retries planning in case of faillure, etc

# ROS imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from gripper_msgs.srv import MoveArm, SetArmGoal, GetArmPosition

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
        

    ##  ------------- SERVICE CALLBACKS ------------- ##
    def set_goal_service_callback(self, request: SetArmGoal.Request, response: SetArmGoal.Response):
        """Callback function for the arm goal service. Uses the list stored
            in request.ee_position to build a pose message and move the arm"""
        self.get_logger().info(f"Moving the arm to {request.goal_ee_location}")

        response.result = True
        return response
    
    def get_position_service_callback(self, request: GetArmPosition.Request, response: GetArmPosition.Response):
        """Callback function for the position request service. Sends a PoseStamped
        message of the current end effector position in the desired frame"""

        self.get_logger().info("returning end effector position")
        
        dummy_pose = PoseStamped()
        dummy_pose.header.frame_id = request.frame
        response.current_ee_pose = dummy_pose
        return response

    
    ##  ------------- SERVICE CLIENT CALLS ------------- ##



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