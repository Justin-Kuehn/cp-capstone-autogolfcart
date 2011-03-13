/* Node.js server for testing */
var net = require('net'), http = require('http'), sys = require('sys');

net.createServer(function(socket) {
	socket.addListener("connect", function(req, res) {
		setInterval( function() {
    		var wheel = Math.floor(Math.random()*60) - 30;
    		var speed = Math.floor(Math.random()*2);
			var data = '{"wheel":'+wheel+', "speed":'+speed+'}';
			socket.write(data);
			sys.puts('Sent '+data);
		},200);
	});
}).listen(50000, "localhost");
