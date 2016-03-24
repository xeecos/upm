// Load makeblock module
var mk = require('jsupm_makeblock');

// Create the sound sensor object using RJ25 port 7
var soundsensor = new mk.MeSoundSensor(7);

//main loop
function loop() {
	//Read the MeSoundSensor value and print soundsensor value
    console.log("sound value = "+soundsensor.read());
}
//Waiting one hundred millisecond to read the loop
setInterval(loop, 100);