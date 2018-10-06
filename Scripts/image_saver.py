#!/usr/bin/env python

IMAGE_TOPIC = '/webcam/image_raw/compressed'

import os
import rospy
from sensor_msgs.msg import CompressedImage

def callback(msg_in):
	ts = str(msg_in.header.stamp.secs) + '-' + str(msg_in.header.stamp.nsecs)

	fmt = ""
	if "png" in msg_in.format:
		fmt = "png"
	elif ("jpeg" in msg_in.format) or ("jpg" in msg_in.format):
		fmt = "jpg"
	else:
		rospy.logwarn("Ignoring image, unkown format: %s" % str(msg_in.format) )

	if fmt:
		filename = 'image_' + ts + '.' + fmt

		rospy.logdebug("saving: " + filename)

		f = open(filename, 'w')
		f.write(msg_in.data)
		f.close()

if __name__ == '__main__':
  rospy.init_node('saver_py')
  rospy.Subscriber(IMAGE_TOPIC, CompressedImage, callback)

  rospy.loginfo("Started image saver node")
  rospy.spin()
