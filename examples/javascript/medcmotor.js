/*dcmotor support RJ25 port 9 and 10*/
// Load makeblock module
var mk = require('jsupm_makeblock');

// Create the dcmotor object using RJ25 port 9(using DIO pin and PWM pin)
var dcmotor = new mk.MeDCMotor(9);

//one second change value
var flag = 0;

//main loop
function loop() {	
   //motor run speed value: between -255 and 255.
   dcmotor.run(flag?100:-100);
   //change motor turn direction
   flag = 1 - flag;
}

// Waiting for two second to read the loop
setInterval(loop, 2000);