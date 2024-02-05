#!/usr/bin/env python3
import rospy
import time
import actionlib
from exp_assignment2.msg import markerInfo, killAll
from exp_assignment2.msg import findMarkerFeedback, findMarkerResult, findMarkerAction
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

"""
	This node is the action server which executes the action of searching a marker
	
	Publishers:   - /cmd_vel :		Linear/angular velocity the robot should have
	              - /requestMarkerId :	Marker id (to find the right one among all markers)
	            
	Subscribers:  - /markerDistance : 	Goal position, taken from marker detection
	
	Actions:      - /findMarkerAction :	Action server that implements the research of a marker
"""

class FindMarker(object):	

	# Constructor
	def __init__(self, name):
		self._action_name = name 
		self._feedback = findMarkerFeedback()
		self._result = findMarkerResult()

		# Attributes
		self.ack = False
		self.to_found = 0
		self.cmd = Twist()

		# Action Server
		self._as = actionlib.SimpleActionServer("/findMarkerAction",
												findMarkerAction,
												execute_cb= self.execute_callback,
												auto_start= False)

		# Publisher for /cmd_vel
		self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		# Subscribe to the topic for goal position from marker detection
		self.sub_marker_dist = rospy.Subscriber("/markerInfo", markerInfo, self.clbk_vision)
		
		self.sub_kill = rospy.Subscriber("/kill_nodes", killAll, self.clbk_kill)

		# Publish for /requestMarkerId
		self.pub_marker_id = rospy.Publisher("/requestMarkerId", Int32, queue_size=10)

		# Start the server
		self._as.start()
		rospy.loginfo("/findMarkerAction server ready")

	# Callback for markerPose subscription
	def clbk_vision(self, msg):
		if(msg.marker_id == self.to_found):
		
			# Acknowledge marker detection
			self.ack = msg.ack			
		else:
		
			#wrong/old marker found 
			self.ack = False

	def clbk_kill(self, msg):
		rospy.signal_shutdown("")

	# Callback function for executing the action
	def execute_callback(self, goal):

		rospy.loginfo("Start the action for searching the target marker: %d" % goal.markerId)
		starting_time = time.time()

		# Send the actual desired markerId
		self.to_found = goal.markerId
		self.pub_marker_id.publish(self.to_found)
		self.ack = False

		while(not(self.ack)):
		
			# While the marker is not found, we make the robot rotate on itself
			self.cmd.angular.z = 0.4
			self.cmd_pub.publish(self.cmd)
			
			# We compute the elapsed time in order to publish it as action feedback
			self._feedback.time_elapsed = time.time() - starting_time
			self._as.publish_feedback(self._feedback)

		# When the marker is found, the robot stops its rotation
		self.cmd.angular.z = 0
		self.cmd_pub.publish(self.cmd)

		# The success of the research is set to True and sent as action result
		self._result.found = True
		rospy.loginfo('Action Succeeded, markerId : %d founded' % goal.markerId)
		self._as.set_succeeded(self._result)

if __name__ == '__main__':

	# Create the node and a FindMarker object
	rospy.init_node('findMarker')
	server = FindMarker(rospy.get_name())
	
	# Spin the node
	rospy.spin()
