#
# Autonomous Golf Cart GUI
# Jason Young 2011
#

# system imports
import sys, socket, time, signal, threading
from PyQt4.QtCore import QUrl
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from mainwindow import Ui_MainWindow		

# my imports
import backend, googlemap

#
# Class MainWindow
#	
status = {
	'speed' : 0,
	'wheel' : 0,
	'latitude' : 0,
	'longitude' : 0,
	'heading' : 0,
	'x' : 0,
	'y' : 0
}
backend = backend.Backend()
class MainWindow(QMainWindow, Ui_MainWindow):
	prev_sends = ""
	prev_recvs = ""
	
	connected = False
	
	defaultmap = "http://maps.google.com/maps?ie=UTF8&ll=35.300215,-120.660439&spn=0.00641,0.017982&z=17&output=embed"
	
	def __init__(self):
		QMainWindow.__init__(self)	
		self.setupUi(self)
	  	
	  	self.googlemap = googlemap.MapHtmlGenerator()
		
		# Bind events
		# Buttons
		QObject.connect(self.connect, SIGNAL("released()"), self.action_connect)
		QObject.connect(self.disconnect,SIGNAL("released()"), self.action_disconnect)
	  	QObject.connect(self.stop, SIGNAL("released()"), self.action_stop)
	  	QObject.connect(self.lookup, SIGNAL("released()"), self.action_lookup)
	  	QObject.connect(self.set_destination, SIGNAL("released()"), self.map_lookup)
		
		# Sliders
		QObject.connect(self.speed, SIGNAL("valueChanged(int)"), self.action_speed)
		QObject.connect(self.steering, SIGNAL("valueChanged(int)"), self.action_wheel)
	
		# Menu
		QObject.connect(self.action_exit, SIGNAL("triggered()"), self.exit)
		QObject.connect(self.action_about, SIGNAL("triggered()"), self.about)
		QObject.connect(self.action_map, SIGNAL("triggered()"), self.reset_map)
		
		#Timers
		self.get_timer = QTimer()
		QObject.connect(self.get_timer, SIGNAL("timeout()"), self.get_status)
		
		#Threads
		self.sender = Send()
		self.receive = Receive()
		
	def get_status(self):
		global status
		if not status:
			return
		if self.show_receives.isChecked() and self.prev_sends != str(status):
			self.console.append('<< ' + str(status))
			
		self.prev_sends = str(status)
		if 'latitude' in status and status['latitude'] is not status['latitude']:
			self.map_lookup(status['latitude'], status['longitude'])
		
		# Ensure stuff for 'security'
		if 'speed' in status:
			status['speed'] = int(status['speed'])
		if 'wheel' in status:
			status['wheel'] = int(status['wheel'])
		if 'latitude' in status:
			status['latitude'] = int(status['latitude'])
		if 'longitude' in status:
			status['longitude'] = int(status['longitude'])
		if 'heading' in status:
			status['heading'] = int(status['heading'])
		if 'x' in status:
			status['x'] = int(status['x'])
		if 'y' in status:
			status['y'] = int(status['y'])
		
		self.status_wheel.setText( str(status['wheel']))
		self.status_speed.setText( str(status['speed']))
		self.status_latitude.setText( str(status['latitude']))
		self.status_longitude.setText( str(status['longitude']))
		self.status_compass.setText( str(status['heading']))
	
	def action_disconnect(self):
		global backend
		backend.disconnect()
		self.connected = False
		self.set_connection_status(False)
		self.console.append("Disconnected")
		self.get_timer.stop()
		self.sender.stop = True
		self.receive.stop = True
		self.sender.join()
		self.receive.join()
	
	def action_connect(self):
		global backend
		host = str(self.ip.displayText().replace(' ',''))
		port = int(self.port.displayText().replace(' ',''))
		
		if not host or not port:
			self.info("Error", "Invalid values")
			return
		
		result = backend.connect(host,port)
		if not result['success']:
			self.info("Error Connecting", result['message'])
			return
		self.set_connection_status(True)
		self.console.append("Connected to %s on port %s" % (host, port))
		self.get_timer.start()
		self.sender.start()
		self.receive.start()

	def set_connection_status(self, connected):
		self.connected = connected
		self.ip.setEnabled(not self.connected)
		self.port.setEnabled(not self.connected)
		self.connect.setEnabled(not self.connected)
		self.disconnect.setEnabled(self.connected)
		if self.connected:
			self.connection_status_label.setText("Connected")
		else:
			self.connection_status_label.setText("Disconnected")
	
	def action_stop(self):
		global status
		if not self.connected: return
		self.speed.setValue(0)
		self.steering.setValue(0)
		status['wheel'] = 0
		status['speed'] = 0
		if self.show_dbw.isChecked():
			self.console.append(">> speed = 0, wheel = 0")

	def action_speed(self, val):
		global status
		if not self.connected: return
		status['speed'] = val
		if self.show_dbw.isChecked():
			self.console.append(">> speed = %d" % (val))
	
	def action_wheel(self, val):
		global status
		if not self.connected: return
		status['wheel'] = val
		if self.show_dbw.isChecked():
			self.console.append(">> wheel = %d" % val)
		
	def action_lookup(self):
		address = self.address.displayText()
		if not address:
			self.info("Notice", "Please enter an address")
			return
		html = self.googlemap.get_geo_html(address)
		self.map.setHtml(html)
	
	def reset_map(self):
		self.map.load(QUrl(self.defaultmap))
	
	def map_lookup(self, lat = None, lon = None):
		if not lat:
			lat = self.latitude.displayText()
		if not lon:
			lon = self.longitude.displayText()
		
		try:
			latitude = float(lat)
			longitude = float(lon)
		except Exception, e:
			self.info("Error", "[%s] Latitude or longitude is not numeric" % e);
			return
		
		html = self.googlemap.get_map_html(latitude, longitude)
		self.map.setHtml(html)
	
	def exit(self):
		global backend
		if self.connected:
			backend.disconnect()
			self.sender.stop = True
			self.receiver.stop = True
			self.sender.join()
			self.receiver.join()
		sys.exit(-1)
	
	def about(self):
		self.info("About", "Created by Jason Young\n\nCapstone Winter 2011")
	
	# Utilities
	def info(self, title="Notice",val="Error"):
		QMessageBox.information(self, title, val, QMessageBox.Ok)

class Send(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
		self.stop = False	
		
	def run(self):
		global status
		global backend
		while not self.stop:
			if status:
				backend.send_command(status)
			time.sleep(.2)

class Receive(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
		self.stop = False	
		
	def run(self):
		global status
		global backend
		
		while not self.stop:
			tstatus = backend.get_status()
			if tstatus:
				status = tstatus
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
