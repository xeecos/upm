// Load makeblock module
var mk = require('jsupm_makeblock');

// Create the Me7SegmentDisplay object using RJ25 port 3
var seven_segment = new mk.Me7SegmentDisplay(3);

//two second change value
var i = 1;

function init_7segmentdisplay()
{
	seven_segment.set(2,0x40,0xc0);
	seven_segment.clear();
	console.log("init_finish");
}

//main loop
function loop() 
{
	//seven_segment.clear();
	seven_segment.display(i);
	i++;
	if(i == 100){
		i = 1;
	}
}
init_7segmentdisplay();
//Waiting for two second to read the loop
setInterval(loop, 100);