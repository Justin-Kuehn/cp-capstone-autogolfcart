#!/usr/bin/env python

  # PMAD Service.
  # Copyright 2010 Andrew Harris

  #  This program is free software: you can redistribute it and/or modify
  #  it under the terms of the GNU General Public License as published by
  #  the Free Software Foundation, either version 3 of the License, or
  #  (at your option) any later version.

  #  This program is distributed in the hope that it will be useful,
  #  but WITHOUT ANY WARRANTY; without even the implied warranty of
  #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  #  GNU General Public License for more details.

  #  You should have received a copy of the GNU General Public License
  #  along with this program.  If not, see <http://www.gnu.org/licenses/>.

import roslib; roslib.load_manifest('pmad')

from pmad.srv import *
import rospy

import os
import sys
import pmad_python
import serial
import time

def pmad_status_fn(pmad, req):
    return StatusResponse(status=pmad.status())

def pmad_switch_fn(pmad, req):
    return SwitchResponse(result=pmad.switch(req.channel, req.state))

def pmad_server(pmad):
    rospy.init_node('pmad_server')
    rospy.Service('pmad_status', Status, (lambda x: pmad_status_fn(pmad, x)))
    rospy.Service('pmad_switch_control', Switch, (lambda x: pmad_switch_fn(pmad, x)))
    rospy.spin()

if __name__ == '__main__':
    tty = rospy.get_param("tty", "/dev/ttyUSB0")
    pmad = pmad_python.PMADInterface(tty)
    # and now start the server
    pmad_server(pmad)
    # and end
    pmad.close()
