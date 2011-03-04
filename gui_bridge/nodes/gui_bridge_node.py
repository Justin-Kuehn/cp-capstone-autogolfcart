#!/usr/bin/env python
import roslib; roslib.load_manifest('gui_bridge')
import rospy
import socket
from geometry_msgs.msg import Twist, Vector3
from golfcart_encoder.msg import GolfcartEncoder

def gui_bridge():	
	#setup ROS node
	rospy.init_node('gui_bridge')
	pub = rospy.Publisher('cmd_vel', Twist)
	
	#setup socket
	HOST = ''
	PORT = 50011
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.bind((HOST, PORT))
	print 'Waiting for connection on port ', PORT
	s.listen(1)
	conn, addr = s.accept()
	print 'Connected by', addr
	
	rospy.sleep(0.5)	
	
	while not rospy.is_shutdown():
		guiCmd = eval(conn.recv(1024))
		print guiCmd
		wheel_angle = float(guiCmd['wheel']) * 0.0174532925 # radians
		speed = float(guiCmd['speed'])
		
		command = Twist()
		command.linear = Vector3 (wheel_angle,0,0)
		command.angular = Vector3 (0,speed,0)
		pub.publish(command)
		rospy.sleep(0.2) # 5Hz

if __name__ == '__main__':
	try:
		gui_bridge()
	except rospy.ROSInterruptException: pass
