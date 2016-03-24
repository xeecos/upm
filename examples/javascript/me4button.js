// Load makeblock module
var mk = require('jsupm_makeblock');

// Create the me4button object using RJ25 port 7(using AIO pin 7)
var me4button = new mk.Me4Button(7);

//main loop
function loop() {
	// Read the button value
	var key_value = me4button.read();
	switch(key_value)
	{
		case 1:
			 console.log("KEY1 pressed!");
		break;
		case 2:
		     console.log("KEY2 pressed!");
		break;
		case 3:
		     console.log("KEY3 pressed!");
		break;	
		case 4:
		     console.log("KEY4 pressed!");
		break;
		default:
		break;
	}
}
//Waiting ten millisecond between readings
setInterval(loop,10);