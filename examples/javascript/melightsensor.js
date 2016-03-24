// Load makeblock module
var mk = require('jsupm_makeblock');

// Create the light sensor object using RJ25 port 7
var lightsensor = new mk.MeLightSensor(7);

//main loop
function loop() {
	//Read the MeLightSensor value
    console.log(lightsensor.read());
}
//Waiting half a second to read the loop
setInterval(loop, 500);