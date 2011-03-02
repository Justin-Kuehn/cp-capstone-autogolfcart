#!/usr/bin/env python
import roslib; roslib.load_manifest('gui_bridge')
import rospy
import socket
from geometry_msgs.msg import Twist
from golfcart_encoder.msg import GolfcartEncoder

def gui_bridge():	
	#setup ROS node
	rospy.init_node('gui_bridge')
	pub = rospy.Publisher('cmd_vel', Twist)
	#setup socket
	HOST = ''
	PORT = 50007
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.bind((HOST, PORT))
	print 'Waiting for connection on port ', PORT
	s.listen(1)
	conn, addr = s.accept()
	print 'Connected by', addr
	while not rospy.is_shutdown():
		guiCmd = eval(s.recv(1024))
		#do whatever with data
		# ('[0-100],0,0' '0,[ -0.7rads, +0.7rads ], 0) 
		pub.publish('0,0,0', '0,0,0')
		rospy.sleep(0.1) # 10Hz
if __name__ == '__main__':
	try:
		gui_bridge()
	except rospy.ROSInterruptException: pass
