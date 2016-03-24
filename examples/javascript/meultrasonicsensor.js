
// Load makeblock module
var mk = require('jsupm_makeblock');

// Create the ultrasonic sensor object using AIO pin 6
var ultrasonicsensor = new mk.MeUltrasonicSensor(6);

// measure the distance
// waiting half a second between readings
function readUltrasonicSensorValue() {
    console.log(ultrasonicsensor.distanceCm()+"cm");
}
setInterval(readUltrasonicSensorValue, 500);