// Load makeblock module
var mk = require('jsupm_makeblock');

// Create the gyro object using white RJ25 port
var gyro = new mk.MeGyro();

//Gyro initialization
gyro.begin();

//main loop
function loop() {
	//Update gyro data
	gyro.update();
	//Read the gyro value and print gx ,gy ,gz value
	console.log("gx:"+gyro.read(0));
	console.log("gy:"+gyro.read(1));
	console.log("gz:"+gyro.read(2));
}
//Waiting one second to read the loop
setInterval(loop, 1000);
