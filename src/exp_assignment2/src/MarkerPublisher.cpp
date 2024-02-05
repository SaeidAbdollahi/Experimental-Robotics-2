/*****************************
 Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are
 permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of
 conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list
 of conditions and the following disclaimer in the documentation and/or other materials
 provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those of the
 authors and should not be interpreted as representing official policies, either expressed
 or implied, of Rafael Muñoz Salinas.
 ********************************/
/**
 * @file marker_publish.cpp
 * @author Bence Magyar
 * @date June 2014
 * @brief Modified copy of simple_single.cpp to publish all markers visible
 * (modified by Josh Langsfeld, 2014)
 */

#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "exp_assignment2/markerInfo.h"
#include "opencv2/core/mat.hpp"
#include "exp_assignment2/killAll.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <sstream>
#include <iostream>
#include <opencv2/highgui.hpp>

class MarkerDetector {
	
	//Attributes
	private:
		/*------------Aruco------------*/		
		//aruco marker detector used to recognize the markers
		aruco::MarkerDetector mDetector_;
		//vector where all recognized markers are save
		std::vector<aruco::Marker> markers_;
		//aruco camera parameter neeeded to identify marker position
		aruco::CameraParameters camParam_;		
		bool useRectifiedImages = true;
		
		//marker size
		double marker_real_size_ = 0.2;
		double marker_size_ = marker_real_size_;
		/*-----------------------------*/
				
		//CV image decompressed from it_
		cv::Mat inImage_;
		
		//id of the marker to find
		int actual_marker_id_;
		bool POV_window_b_;
		
		//message to publish
		exp_assignment2::markerInfo toSend;
		sensor_msgs::ImagePtr outputMsg;
		
		//ROS pubs/subs
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub_;
		image_transport::Publisher debug_pub_;
		
		//Subscriber to get the camera info 	
		ros::Subscriber cam_info_sub;	
		
		//publisher of markerDistance
		ros::Publisher markerPoseGoal_pub_;
		//subscriber to markerId to find
		ros::Subscriber requestMarkerId_sub_;
		ros::Subscriber killer_sub_;	
		
	public:
		MarkerDetector() : nh_("~"), it_(nh_) {

			//create the subscribe to camera image /camera/rgb/image_raw/compressed
			image_sub_ = it_.subscribe("/camera/color/image_raw",1, &MarkerDetector::image_callback, this);
      
			//Pub debug
			debug_pub_ = it_.advertise("/debug/image_raw", 1);	
			
			//publisher to notify NavLogic about the distance from the marker
			markerPoseGoal_pub_ = nh_.advertise<exp_assignment2::markerInfo>("/markerInfo",10);
			
			//subscriber to know the marker to found
			requestMarkerId_sub_ = nh_.subscribe("/requestMarkerId",1, &MarkerDetector::find_marker_callback, this);

			killer_sub_ = nh_.subscribe("/kill_nodes",1, &MarkerDetector::killer_callback, this);
			
			//Read params	
			nh_.param<bool>("pov_window", POV_window_b_, true);
			
			camParam_ = aruco::CameraParameters();
			
			//Debug window for local running
			if(POV_window_b_){
				//Create window for Robot's Camera View
				cv::namedWindow("Robot Camera", cv::WINDOW_AUTOSIZE);	
			}	
		}

		void image_callback(const sensor_msgs::ImageConstPtr& msg) {
			ros::Time curr_stamp = msg->header.stamp;
			cv_bridge::CvImagePtr cv_image_ptr; 
    
			try {
				//get CV Image from sensor image
				cv_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
				inImage_ = cv_image_ptr->image;
				
				//get center of the camera Frame
				int width = static_cast<float>(inImage_.cols);
				int height = static_cast<float>(inImage_.rows);

				//Define the center of the camera image
				cv::Point2f camera_center(width/2.0f, height/2.0f);
   
				// clear out previous detection results
				markers_.clear();

				// detect marker
				mDetector_.detect(inImage_, markers_, camParam_, marker_size_, false);
								
				for (std::size_t i = 0; i < markers_.size(); ++i) {
					// draw detected markers on the image for visualization
					markers_[i].draw(inImage_, cv::Scalar(0, 255, 0), 2);
					//if the marker to found is detected
					if(markers_[i].id == actual_marker_id_){
						//get center of the marker
						cv::Point2f marker_center = markers_[i].getCenter();
						//draw on the image the center of marker for visualization
						cv::circle(inImage_, marker_center, 1, cv::Scalar(0, 255, 0), 2);
						
						// Define the text and font properties
						std::string text = "Marker Found!";
						cv::Point textPosition((inImage_.cols - text.size() * 15) / 2, inImage_.rows / 2);  // Adjust the position based on text size
						int fontFace = cv::FONT_HERSHEY_SIMPLEX;
						double fontScale = 1.0;
						cv::Scalar fontColor(255, 255, 255);  // White color in BGR format
						int thickness = 2;

						// Add the text to the image
						cv::putText(inImage_, text, textPosition, fontFace, fontScale, fontColor, thickness);
						
						//set msg field
						toSend.ack = 1; 						
						toSend.marker_id = markers_[i].id;
						
						//publish marker information
						markerPoseGoal_pub_.publish(toSend);
					}	
				}

				//Visualize the robot's camera 
				if(POV_window_b_){
					cv::imshow("Robot Camera", inImage_);
					cv::waitKey(1);
				}				
				/*Debug publisher*/
				outputMsg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, inImage_).toImageMsg();
				debug_pub_.publish(outputMsg);				
			}
			
			catch (cv_bridge::Exception& e) {
				ROS_ERROR("cv_bridge exception: %s", e.what());
			}
		}

		/*
			Callback function to retrive the desired marker id
		*/
		void find_marker_callback(const std_msgs::Int32 &msg) {
			/*Update the marker to found once a new one has been published*/
			actual_marker_id_ = msg.data;
			ROS_INFO("The camera is searching for the target marker: %d", msg.data);	
		}
		  
		/*
		        If recived a message of finish work this callback 
		        ends the node
		*/
		void killer_callback(const exp_assignment2::killAll &msg){
		        ros::shutdown();
		}
		
				
};

int main(int argc, char **argv){

	// Init ros Node
	ros::init(argc, argv, "marker_detector");	

	// Create markerDetecor
	MarkerDetector node;
	
	// Spin Node
	ros::spin();
}