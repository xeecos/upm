// Load makeblock module
var mk = require('jsupm_makeblock');

// Create the steppermotor object using RJ25 port 9
var steppermotor = new mk.MeStepperMotor(9);

//Steppermotor speed and acceleration init
function steppermotor_init(){
	//Change these to suit your stepper if you want
	steppermotor.setMaxSpeed(4000);
	steppermotor.setAcceleration(1000);
}

//main loop
function loop() {	
        var pos = steppermotor.currentPosition();
	if(pos ==0){
	    steppermotor.moveTo(2000);
    }else if(pos >=2000){
	    steppermotor.moveTo(0);
	}
   steppermotor.run();
}

steppermotor_init();
setInterval(loop,1);
