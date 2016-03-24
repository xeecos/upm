// Load makeblock module
var mk = require('jsupm_makeblock');

// Create the steppermotor object using RJ25 port 9
var steppermotor = new mk.MeStepperMotor(9);

var flag = 0;
var count = 0;

//Steppermotor speed and acceleration init
function steppermotor_init(){
	//Change these to suit your stepper if you want
	steppermotor.setMaxSpeed(1000);
	steppermotor.setAcceleration(20000);
}

//main loop
function loop() {	
	if(flag == 1)
	{
		steppermotor.run();
	}
	if(flag == 0 || count == 1000)
	{
		steppermotor.move(1000);
		flag = 1;
		count = 0;
	}
	count++;
}
steppermotor_init();
//Waiting for three second to read the loop
setInterval(loop,1);