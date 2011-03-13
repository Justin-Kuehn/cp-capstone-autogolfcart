#
# Autonomous Golf Cart GUI
# Jason Young 2011
#

# system imports
import sys, socket, time, signal
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from mainwindow import Ui_MainWindow		

# my imports
import backend, googlemap

# my events
update_event = pyqtSignal(int, QWidget)

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
	
	defaultmap = QUrl("http://maps.google.com/maps?ie=UTF8&ll=35.300215,-120.660439&spn=0.00641,0.017982&z=17&output=embed")
	
	def __init__(self):
		QMainWindow.__init__(self)	
		self.setupUi(self)
	  	
	  	self.googlemap = googlemap.MapHtmlGenerator()
		self.backend = backend.Backend()
		
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
			
		# Custom signals
		self.timer = QTimer()
		QObject.connect(self.timer, SIGNAL("timeout()"), self.run_update)
	
	def send_status(self):
		if not self.backend.send_command(self.status):
			return
		if self.show_sends.isChecked():
			self.console.append('>> ' + str(self.status))
	
	def run_update(self):
		self.get_status()
		self.send_status()
	
	def get_status(self):
		status = self.backend.get_status()
		if not status:
			return
		if self.show_receives.isChecked():
			self.console.append('<< ' + str(status))
		
		if 'latitude' in status and self.status['latitude'] is not status['latitude']:
			self.map_lookup(status['latitude'], status['longitude'])
		
		# Ensure stuff for 'security'
		if 'speed' in status:
			self.status['speed'] = int(status['speed'])
			print status
		if 'wheel' in status:
			self.status['wheel'] = int(status['wheel'])
			print status
		if 'latitude' in status:
			self.status['latitude'] = int(status['latitude'])
		if 'longitude' in status:
			self.status['longitude'] = int(status['longitude'])
		if 'heading' in status:
			self.status['heading'] = int(status['heading'])
		if 'x' in status:
			self.status['x'] = int(status['x'])
		if 'y' in status:
			self.status['y'] = int(status['y'])
		
		self.status_wheel.setText( str(self.status['wheel']))
		self.status_speed.setText( str(self.status['speed']))
		self.status_latitude.setText( str(self.status['latitude']))
		self.status_longitude.setText( str(self.status['longitude']))
		self.status_compass.setText( str(self.status['heading']))
	
	def action_disconnect(self):
		self.backend.disconnect()
		self.connected = False
		self.set_connection_status(False)
		self.console.append("Disconnected")
		self.timer.stop()
	
	def action_connect(self):
		host = str(self.ip.displayText().replace(' ',''))
		port = int(self.port.displayText().replace(' ',''))
		
		if not host or not port:
			self.info("Error", "Invalid values")
			return
		
		result = self.backend.connect(host,port)
		if not result['success']:
			self.info("Error Connecting", result['message'])
			return
		self.set_connection_status(True)
		self.timer.start(20)
		self.console.append("Connected to %s on port %s" % (host, port))

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
		if not self.connected: return
		self.speed.setValue(0)
		self.steering.setValue(0)
		self.status['wheel'] = 0
		self.status['speed'] = 0
		if self.show_dbw.isChecked():
			self.console.append(">> speed = 0, wheel = 0")

	def action_speed(self, val):
		if not self.connected: return
		self.status['speed'] = val
		if self.show_dbw.isChecked():
			self.console.append(">> speed = %d" % (val))
	
	def action_wheel(self, val):
		if not self.connected: return
		self.status['wheel'] = val
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
		self.map.load(self.defaultmap)
	
	def map_lookup(self, lat = None, lon = None):
		if not lat:
			lat = self.latitude.displayText()
		if not lon:
			lon = self.longitude.displayText()
		
		try:
			latitude = long(lat)
			longitude = long(lon)
		except Exception, e:
			self.info("Error", "[%s] Latitude or longitude is not numeric" % e);
			return
		
		html = self.googlemap.get_html(latitude, longitude)
		self.map.setHtml(html)
	
	def exit(self):
		if self.connected:
			self.backend.disconnect()
		sys.exit(-1)
	
	def about(self):
		self.info("About", "Created by Jason Young\n\nCapstone Winter 2011")
	
	# Utilities
	def info(self, title="Notice",val="Error"):
		QMessageBox.information(self, title, val, QMessageBox.Ok)


# Start main function
def main(argv):
	app = QApplication(argv,True)
	wnd = MainWindow()
	wnd.show()

	app.connect(app, SIGNAL("lastWindowClosed()"), app, SLOT("quit()"))

	sys.exit(app.exec_())

if __name__ == "__main__":
	main(sys.argv)
