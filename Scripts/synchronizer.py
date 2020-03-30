#!/usr/bin/env python

import rospy
import message_filters
from std_msgs.msg import Int32

def callback(gas_a, gas_b):
	rospy.loginfo("Synced Measurements: [%i, %i]", gas_a.data, gas_b.data)

if __name__ == '__main__':
	# Initialize
	rospy.init_node('filter_py')

	ga_sub = message_filters.Subscriber('/emulator/sensors/gas/a', Int32, queue_size=10)
	gb_sub = message_filters.Subscriber('/emulator/sensors/gas/b', Int32, queue_size=10)
	ts = message_filters.ApproximateTimeSynchronizer([ga_sub, gb_sub], 10, 0.1, allow_headerless=True)
	ts.registerCallback(callback)

	# Loop here until quit
		try:
		rospy.loginfo("Started subscriber node...")
		rospy.spin()
	except rospy.ROSInterruptException:
		# Shutdown
		rospy.loginfo("Shutting down subscriber!")
		sp.shutdown()
