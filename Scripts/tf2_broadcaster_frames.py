#!/usr/bin/env python

import math

import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import PoseStamped, TransformStamped

# Global Variables
tfbr = None
tfsbr = None

uav_name = "uavusr"
camera_name = "camera"

def send_tf_camera():
	# Create a static transform that is slightly
	# below the UAV and pointing downwards
	t = TransformStamped()
	t.header.stamp = rospy.Time.now()
	t.header.frame_id = uav_name
	t.child_frame_id = camera_name

	t.transform.translation.x = 0.0
	t.transform.translation.y = 0.0
	t.transform.translation.z = -0.05
	q = tf_conversions.transformations.quaternion_from_euler(0, math.pi, 0)
	t.transform.rotation.x = q[0]
	t.transform.rotation.y = q[1]
	t.transform.rotation.z = q[2]
	t.transform.rotation.w = q[3]

	# Send the static transformation
	tfsbr.sendTransform(t)

"""
This is step typically done by the same program that outputs the pose

def callback_pose( msg_in ):
	# Create a transform at the time
	# from the pose message for where
	# the UAV is in the map
	t = TransformStamped()
	t.header = msg_in.header
	t.child_frame_id = uav_name

	t.transform.translation = msg_in.pose.position
	t.transform.rotation = msg_in.pose.orientation

	# Send the transformation
	tfbr.sendTransform(t)
"""

if __name__ == '__main__':
	rospy.init_node('tf2_broadcaster_frames')

	# Setup pose subscriber
	# This functionality is provided by the emulator
	#rospy.Subscriber('/emulated_uav/pose', PoseStamped, callback_pose)

	# Setup tf2 broadcasters
	#    Broadcaster sends a transformation that
	#    can be found at specific time
	#tfbr = tf2_ros.TransformBroadcaster()
	#    Static broadcaster sends a transformation
	#    that is true for all time
	tfsbr = tf2_ros.StaticTransformBroadcaster()

	send_tf_camera()

	rospy.loginfo("tf2_broadcaster_frames running.")

	try:
		rospy.spin()
	except rospy.exceptions.ROSInterruptException:
		pass
	finally:
		rospy.loginfo("tf2_broadcaster_frames shutting down")
