// Load makeblock module
var mk = require('jsupm_makeblock');

// Create the potentiometer sensor object using RJ25 port 7
var potentiometer = new mk.MePotentiometer(7);

//main loop
function loop() {
	//Read the MePotentiometer value and print potentiometer value
    console.log(potentiometer.read());
}
//Waiting half a second to read the loop
setInterval(loop, 500);