// Load makeblock module
var mk = require('jsupm_makeblock');

// Create the flame sensor object using grey RJ25 port 5
var tftlcd = new mk.MeTftLCD();

/*---string variable function---*/
//Clear the screen with c color 
var clear = "CLS(0);";
//Print newline:\r\n
var println = "\r\n";
//The screen displays in upright way
var upright = "DR0;";
var show_flag = 0;

//main loop
function loop() 
{
	switch(show_flag){
		case 1:
		       print_string();	
		break;
		case 2:
		       print_para();
		break;
		case 3:
		       draw_circle();
		break;
		case 4:
		       draw_rectangle();
		break;
		case 5:
		       draw_solid_rectangle();
		break;
		default:
		       show_flag = 0;
		break;
	}
	show_flag++;
}
//Waiting one second to read the loop
setInterval(loop,3000);

//In x=64,y=104 show string "Hello world"
function print_string()
{
	//tips:64 dot matrix;x=64,y=104;show string is Hello world;color=4(yellow),eg:1(red),3(bule)...
    var string = "DS64(64,104,'Hello world!',4)";
	tftlcd.send(clear);	
	tftlcd.send(upright);	
	tftlcd.send(string);
	tftlcd.send(println);
}

//Print temperature and humidity value to para
function print_para()
{
	//tips:64 dot matrix;x=0,y=0;show string is Temperature
    var tem = "DS64(0,0,'Temperature: ";
	var temperature = "28.12";
	var color = "',4);";
	var hum ="DS64(0,64,'Humidity:       ";
	var humidity = "52.36";
	var para_color = "%',4);";
	tftlcd.send(clear);	  
	tftlcd.send(upright);	
	tftlcd.send(tem);
	tftlcd.send(temperature);
	tftlcd.send(color);
	tftlcd.send(hum);
	tftlcd.send(humidity);
	tftlcd.send(para_color);
	tftlcd.send(println);
}

//Draw circle 
function draw_circle()
{
	//center and radius: x=160 y=120;r=80,3(blue) 
	var center_radius = "CIR(160,120,80,3);";
	tftlcd.send(clear);	
	tftlcd.send(upright);	
	tftlcd.send(center_radius);
	tftlcd.send(println);
}

//Draw rectangle  and show string "make block"
function draw_rectangle()
{
	//tips:24 dot matrix;x=112,y=112;show string is 'make block';1(red)
	var str = "DS24(112,112,'make block',1);";
	//draw rectangle >>Upper left corner:x1=85,y1=45;Lower right corner:x2=235,y2=195;4(yellow)
	var rectangle = "BOX(85,45,235,195,4);";
	tftlcd.send(clear);	
	tftlcd.send(upright);	
	tftlcd.send(str);
	tftlcd.send(rectangle);
	tftlcd.send(println);	
}

//Draw solid rectangle
function draw_solid_rectangle()
{
	var rectangle = "BOXF(85,45,235,195,4);";
	tftlcd.send(clear);	
	tftlcd.send(upright);	
	tftlcd.send(rectangle);
	tftlcd.send(println);
}
