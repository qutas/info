#!/usr/bin/env python

import sys
import os
import rospy
import rostopic
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

global bridge
global use_compressed

def callback_image(msg_in):
	global bridge
	global use_compressed

	try:
		if use_compressed:
			cv_image = bridge.compressed_imgmsg_to_cv2( msg_in, "bgr8" )
		else:
			cv_image = bridge.imgmsg_to_cv2( msg_in, "bgr8" )
			
		ts = str(msg_in.header.stamp.secs) + '-' + str(msg_in.header.stamp.nsecs)
		filename = "image_" + ts + ".png"
		rospy.logdebug("saving: %s" % filename)
		
		cv2.imwrite(filename, cv_image)
	except CvBridgeError as e:
		rospy.loginfo(e)
		return

if __name__ == '__main__':
	global bridge
	global use_compressed
	success = False
	img_sub = None
	img_topic = ""

	rospy.init_node('iamge_saver_py', anonymous=True)
	
	if len(sys.argv) == 2:
		img_topic = sys.argv[1]
	else:
		rospy.logerr("Unable to parse input topic!")
		rospy.logerr("\nUsage:\n\tpython image_saver.py /camera_name/image_topic\n\tpython image_saver.py /camera_name/image_topic_compressed")
		rospy.signal_shutdown("bad args")

	if not rospy.is_shutdown():
		data_type = rostopic.get_topic_type(img_topic, blocking=True)[0]
	
		if data_type == "sensor_msgs/Image":
			img_sub = rospy.Subscriber(img_topic, Image, callback_image)
			use_compressed = False
			success = True
		elif data_type == "sensor_msgs/CompressedImage":
			img_sub = rospy.Subscriber(img_topic, CompressedImage, callback_image)
			
			use_compressed = True
			success = True
		else:
			rospy.logerr("Unsupported message type (%s)" % data_type)

		if success:
			try:				
				bridge = CvBridge()
			
				rospy.loginfo("Started image saver node")
				rospy.spin()
			except:
				pass
			finally:
				if img_sub is not None:
					img_sub.unregister()
