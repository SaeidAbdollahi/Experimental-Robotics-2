#include <unistd.h>
#include "exp_assignment2/killAll.h"
#include "../include/dispatch_interface/MoveToInterface.h"
#include "exp_assignment2/PlanningAction.h"
#include "exp_assignment2/PlanningGoal.h"

/*
 * Description:
 * This node serves as the action interface for the movement actions within our plan.
 * Within this action interface, waypoint-based movement is implemented to occur within the simulation.
 * 
 * Actions:
 * - /reaching_goal:
 *   It facilitates robot navigation towards a goal in an environment with obstacles, utilizing the BUG_0 motion planning algorithm.
 */
namespace KCL_rosplan {

	// Initialization
	MoveToInterface::MoveToInterface(ros::NodeHandle &nh) {
		
	}
	
	void killCallback(const exp_assignment2::killAll::ConstPtr& msg) {
		ros::shutdown();
	}
	
	/*
		This function explicitly implements the movement actions based on the dispatched plan.
		It retrieves parameter values from the plan, determining whether a waypoint name is specified.
		If so, it indicates that the robot needs to move to that waypoint.
		The function then sets the goal position to the marker position in the environment and sends it to the reaching_goal action server.
	*/
	bool MoveToInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		std::cout << "Going from " << msg->parameters[0].value << " to " << msg->parameters[1].value << std::endl;
		
		// Initialization of action client for move_base action
		actionlib::SimpleActionClient<exp_assignment2::PlanningAction> client("/reaching_goal", true);
		exp_assignment2::PlanningGoal goal;
		client.waitForServer();
		
		// If a waypoint name is present in the parameters[1], the goal for the robot
		// is set at the correspondant position of that waypoint
		if(msg->parameters[1].value == "wp0"){
			goal.target_pose.pose.position.x = 0.0;
			goal.target_pose.pose.position.y = 1.0;
			goal.target_pose.pose.orientation.w = 1.0;
			goal.target_pose.header.frame_id = "map";
		}
		else if (msg->parameters[1].value == "wp1"){
			goal.target_pose.pose.position.x = 6.0;
			goal.target_pose.pose.position.y = 2.0;
			goal.target_pose.pose.orientation.w = 1.0;
			goal.target_pose.header.frame_id = "map";
		}
		else if (msg->parameters[1].value == "wp2"){
			goal.target_pose.pose.position.x = 8.0;
			goal.target_pose.pose.position.y = -5.0;
			goal.target_pose.pose.orientation.w = 1.0;
			goal.target_pose.header.frame_id = "map";
		}
		else if (msg->parameters[1].value == "wp3"){
			goal.target_pose.pose.position.x = -3.5;
			goal.target_pose.pose.position.y = -8.0;
			goal.target_pose.pose.orientation.w = 1.0;
			goal.target_pose.header.frame_id = "map";
		}
		else if (msg->parameters[1].value == "wp4"){
			goal.target_pose.pose.position.x = -7.0;
			goal.target_pose.pose.position.y = 2;
			goal.target_pose.pose.orientation.w = 1.0;
			goal.target_pose.header.frame_id = "map";
		}
		
		// The goal is sent to the reaching_goal action server
		client.sendGoal(goal);
		sleep(1);
		client.waitForResult();
		ROS_INFO("Action (%s) status: completed!", msg->name.c_str());
		return true;
	}
}


int main(int argc, char **argv) {

	// Node initialization
	ros::init(argc, argv, "moveto_interface_node", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	
	ros::Subscriber sub1 = nh.subscribe("/kill_nodes",1,KCL_rosplan::killCallback);
	
	// Action Interface initialization
	KCL_rosplan::MoveToInterface move_to_interface(nh);
	move_to_interface.runActionInterface();
	return 0;
}
