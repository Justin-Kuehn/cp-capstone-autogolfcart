#!/usr/bin/env python

  # PMAD Status Publisher.
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
import rospy
import pmad.srv
import pmad.msg

def publisher():
    pub = rospy.Publisher('pmad_status', pmad.msg.Status)
    rospy.init_node('pmad_status', anonymous=True)
    while not rospy.is_shutdown():
        rospy.wait_for_service('pmad_status')
        try:
            status_fn = rospy.ServiceProxy('pmad_status', pmad.srv.Status)
            resp = status_fn()
            resp = map(lambda x: ord(x), resp.status)
            analog0 = resp[0] * 256 + resp[1]
            analog1 = resp[2] * 256 + resp[3]
            analog2 = resp[4] * 256 + resp[5]
            analog3 = resp[6] * 256 + resp[7]
            digital4 = resp[9]
            digital5 = resp[10]
            digital6 = resp[11]
            digital7 = resp[12]
            count = resp[13] * 256 + resp[14]
            msg = pmad.msg.Status(analog_0=analog0, analog_1=analog1, analog_2=analog2, analog_3=analog3,
                                  digital_4=digital4, digital_5=digital5, digital_6=digital6, digital_7=digital7,
                                  pmad_command_count=count)
            pub.publish(msg)
        except rospy.ServiceException, e:
            print "PMAD Status Service call failed: %s" % e
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException: pass
