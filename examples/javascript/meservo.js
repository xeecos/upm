// Load makeblock module
var mk = require('jsupm_makeblock');

// Create the MeServoMotor object using RJ25 port 4(using AIO pin 4)
var servomotor = new mk.MeServoMotor(4,1);

//one second change value
var flag = 0;

//main loop
function loop() {
	if(flag)
	{
		servomotor.run(0);
	}
	else
	{
		servomotor.run(180);
	}
	//change servo turn direction
   flag = 1 - flag;
}
// Wait for two second to read the loop
setInterval(loop, 2000);