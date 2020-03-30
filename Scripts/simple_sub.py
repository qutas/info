import rospy
from geometry_msgs.msg import PoseStamped

def callback_pose(msg_in):
	rospy.loginfo("UAV Position: [%0.2f,%0.2f,%0.2f]" % (msg_in.pose.position.x, msg_in.pose.position.y, msg_in.pose.position.z))

if __name__ == '__main__':
	# Initialize
	rospy.init_node('sub_py')
	pose_sub = rospy.Subscriber('/emulated_uav/pose', PoseStamped, callback_pose)

	# Loop here until quit
	try:
		rospy.loginfo("Started subscriber node...")
		rospy.spin()
	except rospy.ROSInterruptException:
		pose_sub.unregister()
		rospy.loginfo("Shutting down subscriber!")
