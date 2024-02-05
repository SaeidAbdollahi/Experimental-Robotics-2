#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include <cmath>
#include <sstream>
#include <iostream>
#include <unistd.h>
#include "std_srvs/Empty.h"
#include "exp_assignment2/killAll.h"
#include "rosplan_dispatch_msgs/DispatchService.h"

/*
 	This node automatically calls four services of the ROSplan system to generate, solve, and prepare a plan for
    execution in ROS.

    The services include:
	- /rosplan_problem_interface/problem_generation_server: Generates the problem.
	- /rosplan_planner_interface/planning_server: Generates a plan.
	- /rosplan_parsing_interface/parse_plan: Parses the plan.
	- /rosplan_plan_dispatcher/dispatch_plan: Dispatches the plan for execution.
 */

class PlanLauncher {
	
	private:
		
		// Attributes
		ros::NodeHandle nh_;
	
	public:
	
		// Constructor
		PlanLauncher() : nh_("~") {
			
		}
		
		/*
			Function that invokes four services and retrieves the success status of the plan dispatching as a response. 
			This is done to verify whether the procedure executed correctly and if the plan is solvable.
		*/
		bool init_plan() {
		
			// Services declaration
			std_srvs::Empty srv1;
			std_srvs::Empty srv2;
			std_srvs::Empty srv3;
			rosplan_dispatch_msgs::DispatchService srv4;
			
			ros::Publisher kill_pub;
			
			// Flag used to check whether the procedure executed correctly or not
			bool done = false;
			bool goal = false;
		
			// Services initialization
			ros::ServiceClient prob_gen = nh_.serviceClient<std_srvs::Empty>("/rosplan_problem_interface/problem_generation_server");
			ros::ServiceClient prob_solve = nh_.serviceClient<std_srvs::Empty>("/rosplan_planner_interface/planning_server");
			ros::ServiceClient prob_parse = nh_.serviceClient<std_srvs::Empty>("/rosplan_parsing_interface/parse_plan");
			ros::ServiceClient plan_disp = nh_.serviceClient<rosplan_dispatch_msgs::DispatchService>("/rosplan_plan_dispatcher/dispatch_plan");
			
			kill_pub = nh_.advertise<exp_assignment2::killAll>("/kill_nodes",1);
			exp_assignment2::killAll kill;
			
			while (not done) {
			
				// While the procedure is not done, the four services are called in order;
				// between every service call, a sleep(2) is placed, to avoid syncronization errors
				prob_gen.call(srv1);
				sleep(2);
				prob_solve.call(srv2);
				sleep(2);
				prob_parse.call(srv3);
				sleep(2);
				plan_disp.call(srv4);
			
				kill.ack = goal;
				kill_pub.publish(kill);
			
				// Get info about if we succeeded or failed
				done = srv4.response.success;
				goal = srv4.response.goal_achieved;
				
				ROS_INFO("My boolean value: %s", done ? "true" : "false");
				ROS_INFO("My boolean value: %s", goal ? "true" : "false");
				
				sleep(2);
				
				kill.ack = goal;
				kill_pub.publish(kill);
			}
		}
};

int main(int argc, char **argv){
	
	// Init ros Node
	ros::init(argc, argv, "plan_launcher_node");	
	
	// Create PlanLauncher
	PlanLauncher node;
	
	// Call PlanLauncher method
	node.init_plan();
	
}
