// Load makeblock module
var mk = require('jsupm_makeblock');

// Create the joystick object using RJ25 port 7
var joystick = new mk.MeJoystick(7);

function loop() {
	
	//Read 0 is print joystick x value and read 1 is print joystick y value 
	console.log("x = "+joystick.read(0)+","+"y = "+joystick.read(1));
}
//Waiting half of second to read the loop
setInterval(loop, 500);
