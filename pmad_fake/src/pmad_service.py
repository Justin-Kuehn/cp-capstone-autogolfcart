#!/usr/bin/env python

import roslib; roslib.load_manifest("pmad_fake")
import rospy
from pmad.srv import *

def handle_Switch(req):
    print "Got channel: %s, state: %s"%(req.channel, req.state)
    return SwitchResponse()

def pmad_service():
    rospy.init_node("pmad_fake")
    cf = rospy.Service('Switch', Switch, handle_Switch)
    print "pmad_fake ready"
    rospy.spin()

if __name__ == "__main__":
    pmad_service()
