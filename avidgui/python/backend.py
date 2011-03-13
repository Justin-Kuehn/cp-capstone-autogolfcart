#
# Autonomous Golf Cart GUI
# Independent Backend
# Jason Young 2011
#
import socket, sys, readline

class Backend():
	speed = 0
	wheel = 0

	sock = None

	def connect(self, host, port):
		if not port or not host: 
			msg =  "No port or host specified"
			sys.stderr.write(msg+"\n")
			return {'success' : False, 'message' : msg}
		host = host.replace(" ","")
		try:
			self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM, 0)
			self.sock.connect((host,port))
			self.sock.settimeout(.2)
		except Exception, e:
			self.sock.close()
			msg = "%s" % e
			sys.stderr.write(msg+"\n")
			return {'success' : False, 'message' : msg}
		
		return {'success' : True}

	def send_command(self, commands):
		if not self.sock: return False
		try:
			self.sock.send(str(commands))
		except Exception, e:
			return False
		return True
	
	def get_status(self):
		try:
			status = self.sock.recv(5012)
		except Exception, e:
			return False
		try:
			if not status.endswith('}') or not status.startswith('{'):
				sys.stderr.write("Invalid string received from server\n")
				return False
			if status.count('}') > 1:
				status = status[status.rfind('{'):]
			status = eval(status)
		except Exception, e:
			sys.stderr.write("[%s] Error parsing status from server\n" % e)
			return False
		return status
	
	def disconnect(self):
		if self.sock:
			self.sock.close()
	

def main():

	if len(sys.argv) is not 3:
		print "Usage: %s <host> <port>" % sys.argv[0]
		sys.exit(-1)

	host = str(sys.argv[1])
	port = int(sys.argv[2])
	
	backend = Backend()
	conn = backend.connect(host,port)
	if not conn['success']:
		print ""
		sys.exit(-1)

	while 1:
		command = raw_input("Enter command: ")
		
		if command == "help":
			print "'get', 'send', or 'stop'"
		
		if command == "stop":
			break
		
		if command == "send":
			name  = raw_input("Enter commands sep. by commas: ").split(',')
			value = raw_input("Enter values sep by commas: ").split(',')
			commands = {}
			for i in range(0,len(name)):
				temp_name = name[i].strip()
				temp_val  = value[i].strip()
				commands[temp_name] = temp_val
				print temp_name
				print temp_val
			backend.send_command(commands)
			print "Command sent"
		
		if command == "get":
			print backend.get_status()

	backend.disconnect()





if __name__ == "__main__":
	main()

