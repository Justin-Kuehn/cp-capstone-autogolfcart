# Sample server for testing
# Echo server program
import socket, sys, os

HOST = ''                 # Symbolic name meaning all available interfaces
PORT = 50000
if len(sys.argv) > 1:
	PORT = int(sys.argv[1])             # Arbitrary non-privileged port\
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
while True:
	s.listen(1)
	conn, addr = s.accept()
	print 'Connected by', addr, ' on port (%d)' % PORT
	if os.fork() == 0:
		while 1:
			data = conn.recv(1024)
			if len(data):
				print data
		conn.close()
conn.close()
s.close()
