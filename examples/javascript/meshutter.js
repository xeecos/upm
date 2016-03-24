// Load makeblock module
var mk = require('jsupm_makeblock');

// Create the shutter object using RJ25 port 3
var shutter = new mk.MeShutter(3);

//main loop
function loop() {	
	setTimeout(function fon(){
		shutter.focusOn();
	},1000);
	setTimeout(function son(){
		shutter.shotOn();
	},1200);
	setTimeout(function off(){
		shutter.shotOff();
		shutter.focusOff();
	},2000);
}
//Waiting for two second to read the loop
setInterval(loop, 2000);