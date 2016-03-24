// Load makeblock module
var mk = require('jsupm_makeblock');

// Create the linefollower object using AIO pin 3
var linefollower = new mk.MeLineFollower(3);

//main loop
function readLineFollowerValue() {
	//Read the linefollower value
	var value = linefollower.read();
	switch(value)
	{
		case 0:
			 console.log("the car is on the middle of the black line");
		break;
		case 1:
			 console.log("the car is on the black line on the right side");
		break;
		case 2:
			 console.log("the car is on the black line on the left side");
		break;
		case 3:
			 console.log("the car is on the middle of the white line");
		break;
		default:
		break;
	}
}
//Waiting one second to read the loop
setInterval(readLineFollowerValue, 1000);