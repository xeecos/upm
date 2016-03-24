// Load makeblock module
var mk = require('jsupm_makeblock');

// Create the humiture object using AIO pin 6
var humiture = new mk.MeHumiture(6);

/* read the both joystick axis values: */
// waiting half a second between readings
function readHumitureValue() {
	
	//update temperature and humidity sensor data
  humiture.update();
  /*console.log("Read sensor: ");
  switch (check)
  {
    case 0: 
                console.log("OK"); 
                break;
    case -1: 
                console.log("Checksum error"); 
                break;
    case -2: 
                console.log("Time out error"); 
                break;
    default: 
                console.log("Unknown error"); 
                break;
  }*/
	//print humidity and temperature value
	console.log("Humidity:"+humiture.getHumidity()+"Temperature:"+humiture.getTemperature());
}
setInterval(readHumitureValue, 5000);