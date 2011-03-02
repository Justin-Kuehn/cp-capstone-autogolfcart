#!/usr/bin/env python
import roslib; roslib.load_manifest('gui_bridge')
import rospy
from geometry_msgs.msg import Twist
from golfcart_encoder.msg import GolfcartEncoder
def gui_bridge():	
	rospy.init_node('gui_bridge')
	pub = rospy.Publisher('cmd_vel', Twist)
	print "GUI BRIDGE STARTED"
	while not rospy.is_shutdown():
		rospy.wait_for_message('encoder', GolfcartEncoder)
		# recieve from gui here stuff here
		# then do something with i
		pub.publish('0,0,0', '0,0,0')
		rospy.sleep(0.1) # 10Hz
		#rospy.wait_for_message(topic, topic_type, timeout=none)
if __name__ == '__main__':
	try:
		gui_bridge()
	except rospy.ROSInterruptException: pass
