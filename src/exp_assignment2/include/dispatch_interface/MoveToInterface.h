/*
 * Namespace: KCL_rosplan
 * 
 * Class: MoveToInterface
 * 
 * Description:
 * This class is part of the ROSPlan library (namespace KCL_rosplan) and serves as an interface for the MoveTo action.
 * It inherits from the RPActionInterface class, indicating its role in handling ROSPlan action interfaces.
 * The MoveTo action typically involves the robot moving to a specified location.
 * 
 * Constructor:
 * MoveToInterface(ros::NodeHandle &nh);
 * - Takes a ROS NodeHandle reference as a parameter and initializes the MoveToInterface object.
 * 
 * Methods:
 * - concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg):
 *   Listens to and processes messages from the action_dispatch topic. Overrides the corresponding method in RPActionInterface.
 *   It is responsible for handling the execution of the MoveTo action.
 *   
 */

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <sstream>
#include <iostream>
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "rosplan_action_interface/RPActionInterface.h"
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>

namespace KCL_rosplan {
        class MoveToInterface: public RPActionInterface {
                private:
                                                
                public:
                        /* constructor */
                        MoveToInterface(ros::NodeHandle &nh);
                        
                        /* listen to and process action_dispatch topic */
                        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
        };
}
