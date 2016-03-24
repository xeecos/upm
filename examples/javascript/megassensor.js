// Load makeblock module
var mk = require('jsupm_makeblock');

// Create the gassensor object using RJ25 port 9
var gassensor = new mk.MeGasSensor(7);

//main loop
function loop() {
	//Read the gassensor value and print gassensor value
	console.log("GasSensor:"+gassensor.read());
}
//Waiting one second to read the loop
setInterval(loop, 1000);