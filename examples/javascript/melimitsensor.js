/*this example only for one limit switch*/

// Load makeblock module
var mk = require('jsupm_makeblock');

// Create the limitswitch object using RJ25 port 3
var limitswitch = new mk.MeLimitSwitch(3);

//only print limitswitch value once
var print_flag = 0;

//Print "Start limitswitch Check" string
console.log("Start LimitSwitch Check.");

//main loop
function loop() {
	//Read the limitswitch value
	var limit_value = limitswitch.read(0);
	if(limit_value && print_flag == 1)
	{
		console.log("Down!");
		print_flag = 0;
	}
	else if(!limit_value && print_flag == 0)
	{
		console.log("Up!");
		print_flag = 1;
	}
}
//Waiting ten millisecond to read the loop
setInterval(loop, 10);