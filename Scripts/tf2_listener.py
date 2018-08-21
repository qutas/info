#!/usr/bin/env python

import math

import rospy
import tf2_ros
from std_msgs.msg import Time
from geometry_msgs.msg import TransformStamped

tfBuffer = None
tfln = None

def callback_target_found(msg_in):
	# We recieved a "found" timestamp
	# attempt to find the transformation
	try:
		# Lookup transform from "map" to "target" at time "msg_in.data",
		# and allow for 0.5 seconds to collected any additionally needed data
		t = tfBuffer.lookup_transform("map", "target", msg_in.data, rospy.Duration(0.5))

		# Dump information to screen
		rospy.loginfo("Found target at the following location in the world:")
		rospy.loginfo("[x: %0.2f; y: %0.2f; z: %0.2f]" % (t.transform.translation.x,
														  t.transform.translation.y,
														  t.transform.translation.z))
	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
		rospy.logwarn(e)

if __name__ == '__main__':
	rospy.init_node('tf2_listener')

	# Setup timestamp subscriber for "target found" at a specific timestamp
	sub_found = rospy.Subscriber('/emulated_uav/target_found', Time, callback_target_found)

	# Create a listener
	# This catches all messages sent using TF2
	tfBuffer = tf2_ros.Buffer()
	tfln = tf2_ros.TransformListener(tfBuffer)

	rospy.loginfo("tf2_listener running.")

	try:
		rospy.spin()
	except rospy.exceptions.ROSInterruptException:
		sub_found.unregister()
	finally:
		rospy.loginfo("tf2_listener shutting down")
