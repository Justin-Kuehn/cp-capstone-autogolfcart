  # Robin protocol functions.
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

ROBIN_MAX_DATA = 80

ROBIN_START = 1
ROBIN_LEADIN = 2
ROBIN_DEST = 3
ROBIN_SRC = 4
ROBIN_FLAGS = 5
ROBIN_LEN = 6
ROBIN_DATA = 7
ROBIN_CKSUM = 8

ROBIN_START_CHAR = chr(0xaa)
ROBIN_LEADIN_CHAR = chr(0x99)

robin_state = ROBIN_START
robin_pkt = {}
robin_data_index = 0;

def sendRobinPacket(s, p):
    s.write(str(ROBIN_START_CHAR))
    s.write(str(ROBIN_LEADIN_CHAR))
    s.write(str(chr(p['dest'])))
    s.write(str(chr(p['src'])))
    s.write(str(chr(p['flags'])))
    l = len(p['data'])
    s.write(str(chr(l)))

    cksum = 0
    cksum = (p['dest'] + p['src'] + p['flags'] + l) & 0xff;
    for i in xrange(0, l):
        cksum += p['data'][i]
        s.write(str(chr(p['data'][i])))
    cksum &= 0xff;
    s.write(str(chr(cksum)))
    p['cksum_comp'] = cksum
    return p

def processRobinByte(c):
    global robin_state
    global robin_pkt
    global robin_data_index

    if robin_state == ROBIN_START: 
        robin_pkt = {}
        if (c == ROBIN_START_CHAR):
            robin_state = ROBIN_LEADIN

    elif robin_state == ROBIN_LEADIN:
        if (c == ROBIN_LEADIN_CHAR):
            robin_state = ROBIN_DEST
        else:
            robin_state = ROBIN_START

    elif robin_state == ROBIN_DEST:
        robin_pkt['dest'] = ord(c)
        robin_state = ROBIN_SRC

    elif robin_state == ROBIN_SRC:
        robin_pkt['src'] = ord(c)
        robin_state = ROBIN_FLAGS

    elif robin_state == ROBIN_FLAGS:
        robin_pkt['flags'] = ord(c)
        robin_state = ROBIN_LEN

    elif robin_state == ROBIN_LEN:
        if (ord(c) < ROBIN_MAX_DATA and ord(c) > 0):
            robin_pkt['data'] = [0] * ord(c)
            robin_data_index = 0
            robin_state = ROBIN_DATA
        else:
            robin_state = ROBIN_START

    elif robin_state == ROBIN_DATA:
        robin_pkt['data'][robin_data_index] = ord(c)
        robin_data_index += 1
        if (robin_data_index == len(robin_pkt['data'])):
            robin_state = ROBIN_CKSUM

    elif robin_state == ROBIN_CKSUM:
        cksum = (robin_pkt['dest'] + 
                 robin_pkt['src'] + 
                 robin_pkt['flags'] + 
                 len(robin_pkt['data'])) & 0xff;
        for i in xrange(0, len(robin_pkt['data'])):
            cksum += robin_pkt['data'][i]
        cksum &= 0xff
        robin_pkt['cksum_rx'] = ord(c)
        robin_pkt['cksum_comp'] = cksum
        robin_state = ROBIN_START
        return robin_pkt

    return None

if __name__ == '__main__':
    import sys
    import serial

    def tester():
        pkt = [chr(0xaa), chr(0x99), chr(0x2), chr(0x1), chr(0x0), chr(0x2), chr(0x68), chr(0x69), chr(0xd6)]
        for p in pkt:
            print(processRobinByte(p))

    ser = serial.Serial('/dev/ttyUSB0', 9600)
    while True:
        p = processRobinByte(ser.read())
        if p == None:
            pass
        else:
            if p['cksum_rx'] == p['cksum_comp']:
                print "PASS"
            else:
                print "FAIL"
