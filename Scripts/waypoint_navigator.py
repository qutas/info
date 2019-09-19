#!/usr/bin/env python2

import sys
from math import *

import roslib
roslib.load_manifest('contrail')
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus

from contrail.msg import TrajectoryAction, TrajectoryGoal
from geometry_msgs.msg import Vector3

if __name__ == '__main__':
	# Waypoints arranged X-Y-Z-Yaw
	waypoints = [[0.0,0.0,1.0,0.0],
				 [1.0,0.0,1.0,0.0],
				 [0.0,0.0,1.0,0.0]]

	# Nominal velocity (used to calculate appropriate segment duration)
	nom_lvel = 0.1

	# Setup ROS and wait for contrail to connect
	rospy.init_node('waypoint_navigator', anonymous=True)

	rospy.loginfo("Waiting for contrail to connect...")
	client_base = actionlib.SimpleActionClient("/emulated_uav/mavel/contrail", TrajectoryAction)
	client_base.wait_for_server()

	# Quick check to make sure we should continue (in-case user pressed CTRL+C during wait)
	if not rospy.is_shutdown():
		# Good to go, start mission
		rospy.loginfo("Starting waypoint mission")
		for i in range(len(waypoints)-1):
			rospy.loginfo("Performing segment: %i" % (i+1))

			# Build new goal message
			# https://github.com/qutas/contrail/blob/master/contrail/action/Trajectory.action
			goal_base = TrajectoryGoal()

			# Create this segment using from the i'th set of waypoints
			p = Vector3(x=waypoints[i][0],y=waypoints[i][1],z=waypoints[i][2])
			goal_base.positions.append(p)
			goal_base.yaws.append(waypoints[i][3])

			pn = Vector3(x=waypoints[i+1][0],y=waypoints[i+1][1],z=waypoints[i+1][2])
			goal_base.positions.append(pn)
			goal_base.yaws.append(waypoints[i+1][3])

			# Calculate a guess of the duration using the nominal velocities and distances between waypoints
			# Would be smart to check yaw rotation as well (hint hint)
			dx = pn.x - p.x
			dy = pn.y - p.y
			dz = pn.z - p.z
			dt = sqrt((dx*dx)+(dy*dy)+(dz*dz)) / nom_lvel
			goal_base.duration = rospy.Duration.from_sec(dt)

			# Set a start time to be "start imidiately"
			goal_base.start = rospy.Time(0)

			# Transmit the goal to contrail
			client_base.send_goal(goal_base)

			# If shutdown is issued (eg. CTRL+C), cancel current mission before rospy is shutdown
			rospy.on_shutdown(lambda : client_base.cancel_goal())
			# Wait for the task to be complete
			client_base.wait_for_result()

			# If task did not succeed, something whent wrong, so quit
			if client_base.get_state() != GoalStatus.SUCCEEDED:
				break;

		# Graceful exit
		rospy.loginfo("Waypoint mission complete!")
		rospy.signal_shutdown("complete")
