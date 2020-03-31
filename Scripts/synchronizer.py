#!/usr/bin/env python

import rospy
import message_filters
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage

def callback(msg_in_pose, msg_in_image):
	rospy.loginfo("Synced Measurements:")
	rospy.loginfo(msg_in_pose)
	rospy.loginfo(msg_in_image.format)

if __name__ == '__main__':
	# Initialize
	rospy.init_node('synchronizer_py')

	sub_pose = message_filters.Subscriber('/emulated_uav/pose', PoseStamped, queue_size=10)
	sub_image = message_filters.Subscriber('/emulated_uav/image/compressed', CompressedImage, queue_size=10)
	ts = message_filters.ApproximateTimeSynchronizer([sub_pose, sub_image], 10, 0.1, allow_headerless=True)
	ts.registerCallback(callback)

	# Loop here until quit
	try:
		rospy.loginfo("Started synchronizer node...")
		rospy.spin()
	except rospy.ROSInterruptException:
		# Shutdown
		rospy.loginfo("Shutting down synchronizer!")
		sp.shutdown()
