#
# Autonomous Golf Cart GUI
# Jason Young
# Feb 20, 2011
#

# Imports
import sys, socket, threading, time
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from mainwindow import Ui_MainWindow		

import backend

#
# V forward velocity 1 or 0
# W angle range [-30, 30]
#
#
# Class MainWindow
#	
class MainWindow(QMainWindow, Ui_MainWindow):
	status = {
		'speed' : 0,
		'wheel' : 0,
		'latitude' : 0,
		'longitude' : 0,
		'heading' : 0,
		'x' : 0,
		'y' : 0
	}
	
	connected = False
	
	def send_status(self):
		self.backend.send_command(self.status)
		self.console.append('Sent ' + str(self.status))
	def __init__(self):
		QMainWindow.__init__(self)	
		self.setupUi(self)
	  
		self.backend = backend.Backend()

		# Bind events
		# Buttons
		QObject.connect(self.connect, SIGNAL("released()"), self.action_connect)
		QObject.connect(self.disconnect,SIGNAL("released()"), self.action_disconnect)
	  	QObject.connect(self.stop, SIGNAL("released()"), self.action_stop)
		
		# Sliders
		QObject.connect(self.speed, SIGNAL("valueChanged(int)"), self.action_speed)
		QObject.connect(self.steering, SIGNAL("valueChanged(int)"), self.action_wheel)
	
		# Menu
		QObject.connect(self.action_exit, SIGNAL("triggered()"), self.exit)
		QObject.connect(self.action_about, SIGNAL("triggered()"), self.about)
	
	def update_status(self):
		self.status_wheel.setText( self.status['wheel'])
		self.status_speed.setText( self.status['speed'])
		self.status_latitude.setText( self.status['latitude'])
		self.status_longitude.setText( self.status['longitude'])
		self.status_compass.setText( self.status['heading'])
		
	
	def action_disconnect(self):
		self.backend.disconnect()
		self.connected = False
		self.set_connection_status()
		self.console.append("Disconnected")
		self.receiver.join()
		self.sender.join()
	
	def action_connect(self):
		host = str(self.ip.displayText())
		port = int(self.port.displayText())
			
		if not host or not port:
			self.info("Error", "Invalid values")
			return
		
		result = self.backend.connect(host,port)
		if not result['success']:
			self.info("Error Connecting", result['message'])
			return
		self.connected = True
		self.set_connection_status()
		
		self.console.append("Connected to %s on port %s" % (host, port))
		
		self.receiver = Receiver(self)
		self.receiver.start()
		self.sender = Sender(self)
		self.sender.start()

	def set_connection_status(self):
		self.ip.setEnabled(not self.connected)
		self.port.setEnabled(not self.connected)
		self.connect.setEnabled(not self.connected)
		self.disconnect.setEnabled(self.connected)
		if self.connected:
			self.connection_status_label.setText("Connected")
		else:
			self.connection_status_label.setText("Disconnected")
	
	def action_stop(self):
		if not self.connected: return
		self.speed.setValue(0)
		self.steering.setValue(0)
		self.status['wheel'] = 0
		self.status['speed'] = 0
		self.console.append("speed = 0, wheel = 0")

	def action_speed(self, val):
		if not self.connected: return
		self.status['speed'] = val
		self.console.append("speed = %d" % (val))
	
	def action_wheel(self, val):
		if not self.connected: return
		self.status['wheel'] = val
		self.console.append("wheel = %d" % val)
	
	def exit(self):
		sys.exit(-1)
	
	def about(self):
		self.info("About", "Created by Jason Young\nCapstone Winter 2011")
	
	# Utilities
	def info(self, title="Notice",val="Error"):
		QMessageBox.information(self, title, val, QMessageBox.Ok)


class Receiver(threading.Thread):
	mainWindow = None
	def __init__(self, mainWindow):
		threading.Thread.__init__(self)
		self.mainWindow = mainWindow
	
	def run(self):
		while True:
			if not self.mainWindow.connected:
				break
			status = self.mainWindow.backend.get_status()
			self.mainWindow.update_status()
			time.sleep(.2)
	
class Sender(threading.Thread):
	mainWindow = None
	def __init__(self, mainWindow):
		threading.Thread.__init__(self)
		self.mainWindow = mainWindow
	
	def run(self):
		while True:
			if not self.mainWindow.connected:
				break
			self.mainWindow.send_status()
			time.sleep(.2)


# Start main function
def main(argv):
	app = QApplication(argv,True)
	wnd = MainWindow()
	wnd.show()

	app.connect(app, SIGNAL("lastWindowClosed()"), app, SLOT("quit()"))

	sys.exit(app.exec_())

if __name__ == "__main__":
	main(sys.argv)
