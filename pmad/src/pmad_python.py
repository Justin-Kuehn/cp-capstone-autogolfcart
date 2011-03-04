  # Basic PMAD functions.
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

import sys
import serial
import robin
import time
import threading

class PMADInterface:
    def __init__(self, tty):
        self.serial_port_lock = threading.Lock()
        # open serial port to PMAD
        self.ser = serial.Serial(tty, 9600)
        time.sleep(10)

    def close(self):
        self.ser.close()

    def _request(self, p):
        # ensure only one request is going at a time
        self.serial_port_lock.acquire()
        # send the packet
        robin.sendRobinPacket(self.ser, p)
        # await robin response
        opcode = None
        rc = None
        data = ""
        while True:
            p = robin.processRobinByte(self.ser.read())
            if p != None:
                # got a packet.  check the checksum
                if p['cksum_rx'] == p['cksum_comp']:
                    pkt = p['data']
                    opcode = pkt[0]
                    if (len(pkt) > 3):
                        rc = pkt[1]
                        data = pkt[2:]
                break
        # release the lock
        self.serial_port_lock.release()
        return (opcode, rc, data)

    def status(self):
        # create the status packet definition.  
        # This is a ROBIN packet as explained in
        # the arduino sketch in arduino/
        p = {'dest':0x2, 'src':0, 'flags':0, 'data':[2]}
        (opcode, rc, data) = self._request(p)
        status_str = ""
        if (opcode == 2 and rc == 0):
            for d in data:
                status_str += chr(d)
        return status_str

    def switch(self, chan, state):
        # create the switch packet definition.  
        # This is a ROBIN packet as explained in
        # the arduino sketch in arduino/
        p = {'dest':0x2, 'src':0, 'flags':0, 'data':[4, chan, state]}
        (opcode, rc, data) = self._request(p)
        switch_res = 1
        if (opcode == 4):
            # got a successful response
            switch_res = 0;
        return switch_res

if __name__ == '__main__':
    pmad = PMADInterface('/dev/ttyUSB1')
    print list(pmad.status())
    print pmad.switch(4, 0)
    print list(pmad.status())
    time.sleep(10)
    print pmad.switch(4, 1)
    pmad.close()
