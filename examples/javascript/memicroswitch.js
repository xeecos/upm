/*this example only for two micro limit switch of RJ25 Adapter*/

// Load makeblock module
var mk = require('jsupm_makeblock');

// Create the microlimitswitch object  using RJ25 port 3
var microlimitswitch = new mk.MeLimitSwitch(3);

//only print microlimitswitch value once
var print_flag0 = 1;
var print_flag1 = 1; 

//Print "Start microlimitswitch Check" string
console.log("Start microlimitswitch Check.");

//main loop
function loop() {
	// read the microlimitswitch value
	var microlimitswitch0 = microlimitswitch.read(0);
	var microlimitswitch1 = microlimitswitch.read(1);
	if(microlimitswitch0 && print_flag0 == 1)
	{
		console.log("SLOT1Up!");
		print_flag0 = 0;
	}
	else if(!microlimitswitch0 && print_flag0 == 0)
	{
		console.log("SLOT1Down!");
		print_flag0 = 1;
	}
	if(microlimitswitch1 && print_flag1 == 1)
	{
		console.log("SLOT2Up!");
		print_flag1 = 0;
	}
	else if(!microlimitswitch1 && print_flag1 == 0)
	{
		console.log("SLOT2Down!");
		print_flag1 = 1;
	}
}
//Waiting ten millisecond to read the loop
setInterval(loop, 10);