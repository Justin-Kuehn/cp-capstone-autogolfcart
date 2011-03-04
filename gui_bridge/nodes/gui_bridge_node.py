#!/usr/bin/env python
import roslib; roslib.load_manifest('gui_bridge')
import rospy
import socket
from geometry_msgs.msg import Twist, Vector3
from golfcart_encoder.msg import GolfcartEncoder

def gui_bridge():	
	#setup ROS node
	rospy.init_node('gui_bridge')
	pub = rospy.Publisher('/golfcart_pilot/abs_cmd', Twist)
	
	#setup socket
	HOST = ''
	PORT = 50012
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.bind((HOST, PORT))
	print 'Waiting for connection on port ', PORT
	s.listen(1)
	conn, addr = s.accept()
	print 'Connected by', addr
	
	rospy.sleep(0.5)	
	
	while not rospy.is_shutdown():
		try:
			data = conn.recv(1024)
			if data.count("{") > 1:
				data = data.split("}{")
				data = "{"+data[len(data)-1]
			guiCmd = eval(data)
			print guiCmd
			wheel_angle = float(guiCmd['wheel']) * 0.0174532925 # d2r
			speed = float(guiCmd['speed'])
		
			command = Twist()
			command.linear = Vector3 (speed,0,0)
			command.angular = Vector3 (0,wheel_angle,0)
			pub.publish(command)
			rospy.sleep(0.2) # 5Hz
		except socket.error:
			command = Twist()
			command.linear = Vector3 (0,0,0)
			command.angular = Vector3 (0,0,0)
			pub.publish(command)
			print 'gui_bridge: SOCKET ERROR OCCURED, STOPPING DRIVE OPERATIONS'
			break
			
	s.close()

if __name__ == '__main__':
	try:
		gui_bridge()
	except rospy.ROSInterruptException: pass
