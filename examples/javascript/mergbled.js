// Load makeblock module
var mk = require('jsupm_makeblock');

// Create the rgbled object using RJ25 yellow port 
var rgbled = new mk.MeRGBLed(2);

//main loop
function loop() {	
	rgbled.setColorAt(1,255,255,255);
}

// Waiting for two second to read the loop
setInterval(loop, 2000);