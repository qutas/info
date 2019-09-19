#!/usr/bin/env python2

import sys
from math import *

import roslib
roslib.load_manifest('contrail')
import rospy
import actionlib
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatus

from contrail.msg import TrajectoryAction, TrajectoryGoal
from geometry_msgs.msg import Vector3

class AsyncNavigator():
	def __init__(self):
		# Internal counter to see what waypoint were are up to
		self.waypoint_counter = 0

		# Waypoints arranged X-Y-Z-Yaw
		self.waypoints = [[0.0,0.0,1.0,0.0],
						  [1.0,0.0,1.0,0.0],
						  [0.0,0.0,1.0,0.0]]

		# Nominal velocity (used to calculate appropriate segment duration)
		self.nom_lvel = 0.1

		# Wait for contrail to connect
		rospy.loginfo("Waiting for contrail to connect...")
		self.client_base = actionlib.SimpleActionClient("/emulated_uav/mavel/contrail", TrajectoryAction)
		self.client_base.wait_for_server()

		if not rospy.is_shutdown():
			# Good to go, start mission
			rospy.loginfo("Starting waypoint mission")

			# Setup first waypoint segment
			self.send_next_wp()

			# Setup a timer to check the goal state at 20Hz
			self.timer = rospy.Timer(rospy.Duration(1.0/20.0), self.check_waypoint_status )
			self.sub = rospy.Subscriber("/chatter", String, self.other_callback)

	# Helper function to current cancel goal if node is shutting down
	def shutdown(self):
		self.client_base.cancel_goal()

	def send_next_wp(self):
		# Build new goal message
		# https://github.com/qutas/contrail/blob/master/contrail/action/Trajectory.action
		goal_base = TrajectoryGoal()

		# Create this segment based on the current counter
		p = Vector3(x=self.waypoints[self.waypoint_counter][0],
					y=self.waypoints[self.waypoint_counter][1],
					z=self.waypoints[self.waypoint_counter][2])
		y = self.waypoints[self.waypoint_counter][3]

		pn = Vector3(x=self.waypoints[self.waypoint_counter+1][0],
					 y=self.waypoints[self.waypoint_counter+1][1],
					 z=self.waypoints[self.waypoint_counter+1][2])
		yn = self.waypoints[self.waypoint_counter+1][3]

		goal_base.positions.append(p)
		goal_base.yaws.append(y)
		goal_base.positions.append(pn)
		goal_base.yaws.append(yn)

		# Calculate a guess of the duration using the nominal velocities and distances between waypoints
		# Would be smart to check yaw rotation as well (hint hint)
		dx = pn.x - p.x
		dy = pn.y - p.y
		dz = pn.z - p.z
		dt = sqrt((dx*dx)+(dy*dy)+(dz*dz)) / self.nom_lvel
		goal_base.duration = rospy.Duration.from_sec(dt)

		# Set a start time to be "start imidiately"
		goal_base.start = rospy.Time(0)

		# Transmit the goal to contrail
		self.client_base.send_goal(goal_base)
		# and message to have some nicer feedback
		rospy.loginfo("Sending waypoint: [%0.2f;%0.2f;%0.2f] -> [%0.2f;%0.2f;%0.2f]" %
					  (p.x,p.y,p.z,pn.x,pn.y,pn.z))

		# Increment our waypoint counter
		self.waypoint_counter += 1

	# We recieve a timer event (te) from rospy.Timer()
	def check_waypoint_status(self, te):
		# If the last segment has succeeded.
		# For more complex tasks, it might be necessary to also
		# check if you are in waypoint or diversion mode here.
		# Hint: you would ignore a succeeded if in diversion mode.
		# Hint: really, we should check for other status states
		#		(such as aborted), as there are some states
		#		where we won't recover from, and should just exit
		if self.client_base.get_state() == GoalStatus.SUCCEEDED:
			if self.waypoint_counter < (len(self.waypoints)-1):
				self.send_next_wp()
			else:
				# Else the mission is over, shutdown and quit the node
				rospy.loginfo("Mission complete!")
				rospy.signal_shutdown("complete")

	def other_callback(self, msg_in):
		# Print the message that we got on our
		# callback just for something to do
		rospy.loginfo(msg_in)

if __name__ == '__main__':
	# Setup ROS
	rospy.init_node('waypoint_navigator_async', anonymous=True)

	try:
		# Setup the navigator
		nav = AsyncNavigator()

		# If shutdown is issued (eg. CTRL+C), cancel current
		# mission before rospy is shutdown.
		rospy.on_shutdown( lambda : nav.shutdown() )
		# Loop here until quit
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("Navigator shutting down")
