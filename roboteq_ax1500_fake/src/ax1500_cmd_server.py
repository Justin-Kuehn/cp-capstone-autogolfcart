#!/usr/bin/env python

import roslib; roslib.load_manifest("roboteq_ax1500_fake")
import rospy
from roboteq_ax1500.srv import *

def handle_channel_forward(req):
    print "Got channel: %s, value: %s"%(req.channel, req.value)
    return channel_forwardResponse()

def ax1500_cmd_server():
    rospy.init_node("ax1500_cmd_server_fake")
    cf = rospy.Service('channel_forward', channel_forward, handle_channel_forward)
    print "ax1500_cmd_server_fake ready"
    rospy.spin()

if __name__ == "__main__":
    ax1500_cmd_server()
