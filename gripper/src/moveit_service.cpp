// This node plans and executes the arm movement with MoveIt2

// Include the basic ROS 2 stuff.
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.h>
#include <geometry_msgs/msg/pose.h>
// Include our custom message type definition.
#include <gripper_msgs/srv/move_arm.hpp>

#include <string>

// The service callback is going to need two parameters, so we declare that
// we're going to use two placeholders.
using std::placeholders::_1;
using std::placeholders::_2;

using moveit::planning_interface::MoveGroupInterface;

int move_arm(std::shared_ptr<gripper_msgs::srv::MoveArm::Request> request){
	// Create MoveIt node
	std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("move_arm_to",
    	rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

	// Create a ROS logger
 	auto const logger = rclcpp::get_logger("move_arm_to");
	RCLCPP_INFO(logger, "created move_arm_to node");

	// Create the MoveIt MoveGroup Interface
	auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
	
	// Set the default reference frame for unspecified Poses to be whatever we got from request
	std::string frame = request->ee_goal.header.frame_id;
	move_group_interface.setPoseReferenceFrame(frame);

	// Set some planning params - 
		// the name of the end effector link
	
	// move_group_interface.setEndEffectorLink("tool0");
		// the maximum time allowed to the planner before aborting
	move_group_interface.setPlanningTime(30.0);
		// the number of times to compute the motion plan before the shortest soln is returned
	move_group_interface.setNumPlanningAttempts(20);

	// Retrieve the pose from the PoseStamped
	auto const target_pose = request->ee_goal.pose;

	RCLCPP_INFO(logger, "making a plan to (%f, %f, %f) in the %s frame", 
		target_pose.position.x, target_pose.position.y, target_pose.position.z, frame.c_str());

	// Create a plan to that target pose
	
	if (request->move_cartesian == true){
		RCLCPP_INFO(logger, "making cartesian path");

		move_group_interface.setEndEffector("suction_gripper");

		std::vector<geometry_msgs::msg::Pose> waypoints;
		waypoints.push_back(target_pose);

		moveit_msgs::msg::RobotTrajectory trajectory;
		double const fraction_complete = move_group_interface.computeCartesianPath(waypoints, 0.001, 0.0, trajectory);
		move_group_interface.execute(trajectory);
	}
	else{
		move_group_interface.setPoseTarget(target_pose, "suction_gripper");

		auto const [success, plan] = [&move_group_interface]{
			moveit::planning_interface::MoveGroupInterface::Plan msg;
			auto const ok = static_cast<bool>(move_group_interface.plan(msg));
			return std::make_pair(ok, msg);
		}();

		// Execute the plan
		if(success) {
			move_group_interface.execute(plan);
		} 
		else {
			RCLCPP_ERROR(logger, "Planing failed!");
		}
	}

	return 0;
}

// The idiom in C++ is the same as in Python; we create a class that
// inherits from the ROS 2 Node class.
class MoveItInterface : public rclcpp::Node {
public:
	MoveItInterface() :Node("moveit_interface") {
		RCLCPP_INFO(this->get_logger(), "Started moveit_interface node");
        // Set up a service to move the arm to a given location.
		service_ = this->create_service<gripper_msgs::srv::MoveArm>("move_arm", std::bind(&MoveItInterface::moveArmCallback, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Started arm service");

	}
private:
    // A variable for the service.
	rclcpp::Service<gripper_msgs::srv::MoveArm>::SharedPtr service_;

    // This is the callback function.  It takes a request and a response.  The
	// callback returns void; results are returned through the response argument.
	void moveArmCallback(const std::shared_ptr<gripper_msgs::srv::MoveArm::Request> request,
		const std::shared_ptr<gripper_msgs::srv::MoveArm::Response> response) const {

		RCLCPP_INFO(this->get_logger(), "Recieved move_arm message %f", request->ee_goal.pose.position.x);

		int planning_result = move_arm(request);
		
        // The result is returned by filling in the response.
		if(planning_result==0){
			response->result = true;
		}
		else{
			response->result = false;
		}
		
	}   
};


// This is the entry point for the executable.  Unlike Python, we
// can only have a single entry point in C++.
int main(int argc, char **argv) {
	// Initialize ROS.
	rclcpp::init(argc, argv);

	// Create a node and give control to the ROS event handler.
	rclcpp::spin(std::make_shared<MoveItInterface>());

	// Once the event handler is done, shut things down nicely.
	rclcpp::shutdown();

	return 0;
}