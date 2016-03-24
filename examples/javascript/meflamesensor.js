// Load makeblock module
var mk = require('jsupm_makeblock');

// Create the flame sensor object using RJ25 port 7
var flamesensor = new mk.MeFlameSensor(7);

//main loop
function loop() {
	//Read the MeFlameSensor value and print flamesensor value
	console.log("FlameSensor:"+flamesensor.read());		  
}
//Waiting one second to read the loop
setInterval(loop,1000);