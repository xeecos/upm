// Load makeblock module
var mk = require('jsupm_makeblock');

// Create the light sensor object using RJ25 port 3
var pirmotionsensor = new mk.MePIRMotionSensor(3);

//main loop
function loop() {
	//Read the MePIRMotionSensor value 
	var value = pirmotionsensor.read();
	if(value)
	{
		console.log("Someone in");
	}
	else
	{
		console.log("No one in");
	}
}
//Waiting one second to read the loop
setInterval(loop, 1000);