#
# Autonomous Golf Cart GUI
# Map Helper Class
# Jason Young 2011
#
class MapHtmlGenerator():
	mapOptions = {
		'zoom' : 18, # 0 is farthest out
		'lat'  : 35.3046,
		'lon'  :-120.6676 
	}
	GEOTMPL = '\
	<!DOCTYPE html><html><head>\
	<meta name="viewport" content="initial-scale=1.0, user-scalable=no"/> \
	<meta http-equiv="content-type" content="text/html; charset=UTF-8"/>\
	<script type="text/javascript" \
	src="http://maps.google.com/maps/api/js?sensor=false"></script><script \
	type="text/javascript">var address="__ADDRESS__";var geocoder;var map;\
	function initialize(){geocoder=new google.maps.Geocoder();codeAddress();}\
	function codeAddress(){geocoder.geocode({"address":address},function(results,\
	status){if(status==google.maps.GeocoderStatus.OK){document.body.innerHTML=\
	"<h3>"+address+"</h3><h4>"+results[0].geometry.location+"</h4>";}else\
	{alert("Geocode was not successful for the following reason: "+status);}});}\
	initialize();</script></head><body style="margin:0;padding:10px 0 0 10px">\
	</body></html>'
	
	HTMLTMPL = '\
	<!DOCTYPE html><html><head>\
	<meta name="viewport" content="initial-scale=1.0, user-scalable=no" />\
	<style type="text/css">html{height:100%}body{height:100%;margin:0px;\
	padding:0px}#map_canvas{height:100% }</style><script \
	type="text/javascript" src="http://maps.google.com/maps/api/js?sensor=false">\
	</script><script type="text/javascript">function initialize(){var latlng=\
	new google.maps.LatLng(__LAT__, __LON__);var myOptions={zoom:__ZOOM__,\
	center:latlng,mapTypeId:google.maps.MapTypeId.ROADMAP};var map=new\
	google.maps.Map(document.getElementById("map_canvas"),myOptions);new\
	google.maps.Marker({position: latlng}).setMap(map);}</script></head>\
	<body onload="initialize()">\
	<div id="map_canvas" style="width:100%; height:100%"></div></body></html>';
	
	def get_geo_html(self, address):
		if not address: return
		address = address.replace("'", "\\'")
		return self.GEOTMPL.replace('__ADDRESS__', address)
		
	def get_map_html(self, lat = None, lon = None):
		if not lat: lat = self.mapOptions['lat']
		if not lon: lon = self.mapOptions['lon']
		lat = str(lat)
		lon = str(lon)
		return self.HTMLTMPL.replace('__LAT__', lat).replace('__LON__', lon).replace('__ZOOM__', str(self.mapOptions['zoom']))
