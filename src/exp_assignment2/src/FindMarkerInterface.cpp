#include "../include/dispatch_interface/FindMarkerInterface.h"
#include "exp_assignment2/killAll.h"
#include <unistd.h>

/*
 * Description:
 * This node functions as the action interface for the find_marker action in our plan.
 * Within this action interface, an action client is implemented to invoke the find_marker action.
 *
 * Actions:
 * - /findMarkerAction:
 *   This action is utilized to search for a marker with a specified ID.
 */

namespace KCL_rosplan {

    // Initialization
    FindMarkerInterface::FindMarkerInterface(ros::NodeHandle &nh) {
    }
    
    //Kill the current node
    void killCallback(const exp_assignment2::killAll::ConstPtr& msg) {
        ros::shutdown();
    }

    /*
        This function specifically implements the detection action by extracting parameter values from the
        dispatched plan. It checks whether a marker ID is present. If so, it signifies that the robot needs
        to rotate to locate the marker. 
        
        Subsequently, the function sets the marker ID as a goal for the find_marker action and proceeds to
        invoke the corresponding action server.
    */
    bool FindMarkerInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        ROS_INFO("Target marker: %s", msg->parameters[1].value.c_str());
        
        // Get the value of parameters[1] and check if it contains a marker ID 
        std::string marker_name = msg->parameters[1].value.c_str();
        int markerID;
        if(marker_name == "mk11"){
            markerID = 11;
        }else if(marker_name == "mk12"){
            markerID = 12;
        }else if(marker_name == "mk13"){
            markerID = 13;
        }else if(marker_name == "mk15"){
            markerID = 15;
        }

        // Set the marker ID found as goal for the find marker action
        exp_assignment2::findMarkerGoal goal;
        goal.markerId = markerID;

        // Action client initialization
        actionlib::SimpleActionClient<exp_assignment2::findMarkerAction> client("/findMarkerAction", true);
        ROS_INFO("Waiting for action server to start.");
        client.waitForServer();    
        ROS_INFO("Action server /findMarkerAction available");
        
        // Sending the marker ID to the action server
        client.sendGoal(goal);
        sleep(1);
        
        // We wait one minute for the completition of the action
        bool res = client.waitForResult(ros::Duration(60.0));
        
        // We check whether the action succeded or failed and then we print a message to let the user know about it
        if(res){
            ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
            return true;
        }else{
            ROS_INFO("Action (%s): failed", msg->name.c_str());
            return false; 
        }
        return true;
    }
}

int main(int argc, char **argv) {

    // Node initialization
    ros::init(argc, argv, "find_marker_interface_node", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    
    ros::Subscriber sub = nh.subscribe("/kill_nodes",1,KCL_rosplan::killCallback);
    
    // Action interface initialization
    KCL_rosplan::FindMarkerInterface find_marker_interface(nh);
    find_marker_interface.runActionInterface();
    return 0;
}
