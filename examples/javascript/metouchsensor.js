// Load makeblock module
var mk = require('jsupm_makeblock');

// Create the metouchsensor object using RJ25 port 3
var touchsensor = new mk.MeTouchSensor(3);

//only print MeTouchSensor value once
var print_flag = 0;

//Print "Start TouchSensor Check" string
console.log("Start TouchSensor Check.");

//main loop
function loop() {
	//Read the MeTouchSensor value
	var touch_value = touchsensor.read();
	if(touch_value && print_flag == 1)
	{
		console.log("Touching!");
		print_flag = 0;
	}
	else if(!touch_value && print_flag == 0)
	{
		console.log("Not Touch!");
		print_flag = 1;
	}		
}
//Waiting ten millisecond to read the loop
setInterval(loop, 10);