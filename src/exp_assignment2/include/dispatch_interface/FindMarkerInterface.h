/*
 * Class: FindMarkerInterface
 * 
 * Description:
 * This class is part of the ROSPlan library (namespace KCL_rosplan) and serves as an interface for detecting markers.
 * It inherits from the RPActionInterface class, indicating its role in handling ROSPlan action interfaces.
 *
 * 
 * Constructor:
 * FindMarkerInterface(ros::NodeHandle &nh);
 * - Takes a ROS NodeHandle reference as a parameter and initializes the FindMarkerInterface object.
 * 
 * Methods:
 * - concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg):
 *   Listens to and processes messages from the action_dispatch topic. Overrides the corresponding method in RPActionInterface.
 *   It is responsible for handling the execution of the detect_marker action.
 *   
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <exp_assignment2/findMarkerAction.h>
#include "rosplan_action_interface/RPActionInterface.h"
#include <unistd.h>
#include <string>

namespace KCL_rosplan {
        class FindMarkerInterface: public RPActionInterface {
                private:
                        ros::ServiceClient clientMarkerVision;                                             
                public:
                        /* constructor */
                        FindMarkerInterface(ros::NodeHandle &nh);
                        
                        /* listen to and process action_dispatch topic */
                        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
        };
}
