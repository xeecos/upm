#include "makeblock.h"
 
#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <functional>

using namespace upm;
#define NC 0
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
#define A6 20
#define A7 21

/**
 * Class: MePort
 *
 * \par Description
 * Declaration of Class MePort
 */
MePort::MePort(){
	 uint8_t list[30] = {NC, NC , 11, 10 ,  9, 12 , 13,  8 , NC,  3 , NC, NC , NC,  2 , A2, A3 , A0, A1 ,  5,  4 ,  6,  7 , NC, NC , NC, NC , NC, NC , NC, NC};
	 ports = (uint8_t*)malloc(30);
	 for(int i=0;i<30;i++)ports[i] = list[i];
}
/**
 * \par Function
 *   getPin
 * \par Description
 *   Get the pin
 * \param[in]
 *   port - RJ25 port from PORT_1 to PORT_10
 * \param[in]
 *   slot - SLOT1 or SLOT2
 * \return
 *   The port pin value
 * \par Others
 *   None
 */
uint8_t MePort::getPin(uint8_t port,uint8_t slot){
	return ports[port*2+slot];
}
/********************
*******Me DC Motor
*********************/
/**
 * Alternate Constructor which can call your own function to map the DC motor to arduino port
 * pin will be used here
 * \param[in]
 *   port - RJ25 port from PORT_9 to PORT_10
 */
MeDCMotor::MeDCMotor(uint8_t port){
	mraa_init();
	MePort pt = MePort();
	uint8_t s1 = pt.getPin(port,0);
	uint8_t s2 = pt.getPin(port,1);
	pin1 = mraa_pwm_init(s1);
    if (pin1 == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s1);
        exit (1);
    }
	pin2 = mraa_gpio_init(s2);
    if (pin2 == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s2);
        exit (1);
    }
	mraa_pwm_enable(pin1,1);
	mraa_gpio_use_mmaped(pin2,1);
	mraa_gpio_dir(pin2, MRAA_GPIO_OUT);
}

/**
 * Uninstall Constructor when you call function  ~MeDCMotor() 
 * \param[in]
 *   None
 */
MeDCMotor::~MeDCMotor (){
    mraa_pwm_close(pin1);
    mraa_gpio_close(pin2);
}

/**
 * \par Function
 *   run
 * \par Description
 *   Control the motor forward or reverse
 * \param[in]
 *   speed - Speed value from -255 to 255
 * \par Output
 *   None
 * \return
 *   None
 * \par Others
 *   None
 */
void MeDCMotor::run(float pwm){
	if(pwm>255){
		pwm = 255;
	}
	if(pwm<-255){
		pwm = -255;
	}
	uint8_t dir = 0;
	if(pwm<0){
		pwm = -pwm;
		dir = 1;
	}
	mraa_pwm_write(pin1,pwm/255.0);
	mraa_gpio_write(pin2,dir);
}
/********************
*******Me Servo Motor
*********************/
/**
 * Alternate Constructor which can call your own function to map the Servo Motor to arduino port
 * \param[in]
 *   port - RJ25 port only PORT_4 and solt is 1
 */	
MeServoMotor::MeServoMotor(uint8_t port,uint8_t slot){
	MePort pt = MePort();
	int s = pt.getPin(port,slot);
    svr = new Servo(s);
	
}
/**
 * Uninstall Constructor when you call function  ~MeServoMotor() 
 * \param[in]
 *   None
 */
MeServoMotor::~MeServoMotor (){
}
/**
 * \par Function
 *   run
 * \par Description
 *   change for the servo motor angle
 * \param[in]
 *   angle - angle value is from 0 to 180
 * \par Output
 *   None
 * \return
 *   None
 * \par Others
 *   None
 */
void MeServoMotor::run(uint8_t angle){
	svr->setAngle(angle);
}
	
/********************
*******Me Stepper Motor
*********************/

typedef enum
{
  DIRECTION_CCW = 0,  ///< Clockwise
  DIRECTION_CW  = 1   ///< Counter-Clockwise
} Direction;
/**
 * Alternate Constructor which can call your own function to map the stepper to arduino port,
 * pins are used or initialized here.
 * \param[in]
 *   port
 */
MeStepperMotor::MeStepperMotor(uint8_t port){
    mraa_init();
	MePort pt = MePort();
	uint8_t s1 = pt.getPin(port,0);
	uint8_t s2 = pt.getPin(port,1);
	pin1 = mraa_gpio_init(s1);
    if (pin1 == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s1);
        exit (1);
    }
	pin2 = mraa_gpio_init(s2);
    if (pin2 == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s2);
        exit (1);
    }
	mraa_gpio_use_mmaped(pin1, 1);
	mraa_gpio_use_mmaped(pin2, 1);
	mraa_gpio_dir(pin1, MRAA_GPIO_OUT);
	mraa_gpio_dir(pin2, MRAA_GPIO_OUT);
	
	_currentPos = 0;
	_targetPos = 0;
	_acceleration = 0;
	gettimeofday(&timer, NULL);
	double currentTime = 1000000 * timer.tv_sec + timer.tv_usec;
	_lastStepTime = currentTime;
	_speed = 0;
	_dir = DIRECTION_CW;
}
/**
 * Uninstall Constructor when you call function  ~MeStepperMotor() 
 * \param[in]
 *   None
 */	
MeStepperMotor::~MeStepperMotor (){
    mraa_gpio_close (pin1);
    mraa_gpio_close (pin2);
}
/**
 * \par Function
 *    step
 * \par Description
 *    Stepper runs step by step.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
void MeStepperMotor::step(){
	mraa_gpio_write(pin1, _dir==DIRECTION_CW?LOW:HIGH);
	mraa_gpio_write(pin2,HIGH);
	usleep(1);
	mraa_gpio_write(pin2,LOW);
}
/**
 * \par Function
 *    run
 * \par Description
 *    Stepper's status----run or not.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \par Return
 *    Return the status.
 * \par Others
 *    None
 */	
bool MeStepperMotor::run(){
	if((_speed == 0.0) || (distanceToGo() == 0))
	{
		return false;
	}

	if (runSpeed())
	{
		computeNewSpeed();
		return true;
	}
}
/**
 * \par Function
 *    ctime
 * \par Description
 *    set stepper interval
 * \param[in]
 *    None
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
unsigned long MeStepperMotor::ctime(){
	gettimeofday(&timer, NULL);
	return 1000000 * timer.tv_sec + timer.tv_usec;
}
/**
 * \par Function
 *    runSpeed
 * \par Description
 *    The speed of Stepper's running.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \par Return
 *    Return true or false.
 * \par Others
 *    None
 */
bool MeStepperMotor::runSpeed()
{
	// Dont do anything unless we actually have a step interval
	if (!_stepInterval)
	{
		return false;
	}
	
    gettimeofday(&timer, NULL);
	unsigned long currentTime = 1000000 * timer.tv_sec + timer.tv_usec;
	if (currentTime - _lastStepTime > _stepInterval)
	{
		if (_dir == DIRECTION_CW)
		{
		  // Clockwise
			_currentPos += 1;
		}
		else
		{
		  // Anticlockwise  
			_currentPos -= 1;
		}
		step();
		_lastStepTime = currentTime;
		return true;
	}
	else
	{
		return false;
	}
}
/**
 * \par Function
 *    currentPosition
 * \par Description
 *    Stepper's current position.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \par Return
 *    Return the current position of Stepper.
 * \par Others
 *    None
 */
long MeStepperMotor::currentPosition(){
	return _currentPos;
}
/**
 * \par Function
 *    distanceToGo
 * \par Description
 *    The distance that Stepper should go.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \par Return
 *    Return the length of Stepper's running.
 * \par Others
 *    None
 */	
long MeStepperMotor::distanceToGo()
{
	return _targetPos - _currentPos;
}
/**
 * \par Function
 *    stepInterval
 * \par Description
 *    set timer vaule
 * \param[in]
 *    None
 * \par Output
 *    None
 * \par Return
 *    time value
 * \par Others
 *    None
 */
unsigned long MeStepperMotor::stepInterval(){
	return _stepInterval;
}
/**
 * \par Function
 *    computeNewSpeed
 * \par Description
 *    compute stepper new speed 
 * \param[in]
 *    None
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
void MeStepperMotor::computeNewSpeed()
{
	long distanceTo = distanceToGo();
	long stepsToStop = (long)((_speed * _speed) / (2.0 * _acceleration));
	if (distanceTo == 0 && stepsToStop <= 1)
	{
		// We are at the target and its time to stop
		_stepInterval = 0;
		_speed = 0.0;
		_n = 0;
		return;
	}

	if (distanceTo > 0)
	{
		// We are anticlockwise from the target
		// Need to go clockwise from here, maybe decelerate now
		if (_n > 0)
		{
			// Currently accelerating, need to decel now? Or maybe going the wrong way?
			if ((stepsToStop >= distanceTo) || _dir == DIRECTION_CCW)
			{
				_n = -stepsToStop; // Start deceleration
			}
		}
		else if (_n < 0)
		{
			// Currently decelerating, need to accel again?
			if ((stepsToStop < distanceTo) && _dir == DIRECTION_CW)
			{
				_n = -_n; // Start accceleration
			}
		}
	}
	else if (distanceTo < 0)
	{
		// We are clockwise from the target
		// Need to go anticlockwise from here, maybe decelerate
		if (_n > 0)
		{
			// Currently accelerating, need to decel now? Or maybe going the wrong way?
			if ((stepsToStop >= -distanceTo) || _dir == DIRECTION_CW)
			{
				_n = -stepsToStop; // Start deceleration
			}
		}
		else if (_n < 0)
		{
			// Currently decelerating, need to accel again?
			if ((stepsToStop < -distanceTo) && _dir == DIRECTION_CCW)
			{
				_n = -_n; // Start accceleration
			}
		}
	}

	// Need to accelerate or decelerate
	if (_n == 0)
	{
		// First step from stopped
		_cn = _c0;
		_dir = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
	}
	else
	{
		// Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
		_cn = _cn - ((2.0 * _cn) / ((4.0 * _n) + 1)); // Equation 13
		_cn = _cn>_cmin?_cn:_cmin;
	}
	_n++;
	_stepInterval = _cn;
	_speed = 1000000.0 / _cn;
	if (_dir == DIRECTION_CCW)
	{
		_speed = -_speed;
	}
}
/**
 * \par Function
 *    setMaxSpeed
 * \par Description
 *    Set Max Speed for Stepper.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
void MeStepperMotor::setMaxSpeed(float speed)
{
  if (_maxSpeed != speed)
  {
    _maxSpeed = speed;
    _cmin = 1000000.0 / speed;
    // Recompute _n from current speed and adjust speed if accelerating or cruising
    if (_n > 0)
    {
      _n = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16
      computeNewSpeed();
    }
  }
}
/**
 * \par Function
 *    setSpeed
 * \par Description
 *    Set Speed for Stepper.
 * \param[in]
 *    speed - The speed of Stepper.
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
void MeStepperMotor::setSpeed(uint16_t speed){
	if (speed == _speed){
		return;
	}
	speed = speed<-_maxSpeed?-_maxSpeed:(speed>_maxSpeed?_maxSpeed:speed);
	if (speed == 0.0){
		_stepInterval = 0;
	}else{
		_stepInterval = fabs(1000000.0 /speed);
		_dir = (speed > 0.0) ? DIRECTION_CW : DIRECTION_CCW;
	}
	_speed = speed;
}
void MeStepperMotor::setAcceleration(float acceleration){
	if(acceleration == 0.0)
	{
		return;
	}
	if(_acceleration != acceleration)
	{
		_n = _n * (_acceleration / acceleration);
		//	_c0 = sqrt(2.0 / acceleration) * 1000000.0;
		// Accelerates at half the expected rate. Why?
		_c0 = sqrt(1.0/acceleration) * 1000000.0;
		_acceleration = acceleration;
		computeNewSpeed();
	}
}
/**
 * \par Function
 *    move
 * \par Description
 *    Stepper moves to the aim.
 * \param[in]
 *    relative - The relative length to Stepper's movement.
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
void MeStepperMotor::move(long distance){
	 moveTo(_currentPos + distance);
}
/**
 * \par Function
 *    moveTo
 * \par Description
 *    Stepper moves to the aim.
 * \param[in]
 *    absolute - The absolute length to Stepper's movement.
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
void MeStepperMotor::moveTo(long distance){
	if (_targetPos != distance)
	{
		_targetPos = distance;
		computeNewSpeed();
	}
}
/********************
*******Me7SegmentDisplay
*********************/
const uint8_t tubeTable[] = 
{
  0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, //0-9
  0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71,                         //'A', 'B', 'C', 'D', 'E', 'F',
  0xbf, 0x86, 0xdb, 0xcf, 0xe6, 0xed, 0xfd, 0x87, 0xff, 0xef, //0.-9.
  0xf7, 0xfc, 0xb9, 0xde, 0xf9, 0xf1,                         //'A.', 'B.', 'C.', 'D.', 'E.', 'F.',
  0, 0x40                                                     //' ','-'
};
/**
 * Alternate Constructor which can call your own function to map the 7-Segment display to arduino port,
 * the slot1 will be used for data pin and slot2 will used for clk pin.
 * \param[in]
 *   port - RJ25 blue port
 */	
Me7SegmentDisplay::Me7SegmentDisplay(uint8_t port){
	mraa_init();
	MePort pt = MePort();
	uint8_t s1 = pt.getPin(port,0);
	uint8_t s2 = pt.getPin(port,1);
	pin1 = mraa_gpio_init(s1);
    if (pin1 == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s1);
        exit (1);
    }
	pin2 = mraa_gpio_init(s2);
    if (pin2 == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s2);
        exit (1);
    }
	mraa_gpio_use_mmaped(pin1, 1);
	mraa_gpio_use_mmaped(pin2, 1);
	mraa_gpio_dir(pin1, MRAA_GPIO_OUT);
	mraa_gpio_dir(pin2, MRAA_GPIO_OUT);
	
	set(BRIGHTNESS_2, ADDR_AUTO, STARTADDR);
	clear();
}
/**
 * Uninstall Constructor when you call function  ~Me7SegmentDisplay() 
 * \param[in]
 *   None
 */
Me7SegmentDisplay::~Me7SegmentDisplay (){
    mraa_gpio_close (pin1);
    mraa_gpio_close (pin2);
}
/**
 * \par Function
 *    setBrightness
 * \par Description
 *    Set brightness.
 * \param[in]
 *    brightness - Brightness, defined in Me7SegmentDisplay.h from BRIGHTNESS_0 to BRIGHTNESS_7.
 * \par Output
 *    Cmd_DispCtrl - Control command for Me 7 Segment Serial Display module.
 * \return
 *    None
 * \par Others
 *    None
 */
void Me7SegmentDisplay::set(uint8_t brightness, uint8_t SetData, uint8_t SetAddr)
{
  Cmd_SetData = SetData;
  Cmd_SetAddr = SetAddr;
  Cmd_DispCtrl = SEGDIS_ON + brightness;//Set brightness, take effect next display cycle.
}
/**
 * \par Function
 *    writeByte
 * \par Description
 *    Write one byte to TM1637.
 * \param[in]
 *    wr_data - Data to write to module.
 * \par Output
 *    None
 * \return
 *    None
 * \others
 *    None
 */
void Me7SegmentDisplay::writeByte(uint8_t wr_data)
{
	uint8_t i;
	uint8_t cnt0;
	for (i = 0; i < 8; i++)  //sent 8bit data
	{
		mraa_gpio_write (pin2,LOW);
		if (wr_data & 0x01)
		{
			mraa_gpio_write (pin1,HIGH); //LSB first
		}
		else
		{
			mraa_gpio_write (pin1,LOW);
		}
		wr_data >>= 1;
		mraa_gpio_write (pin2,HIGH);
	}
	mraa_gpio_write (pin2,LOW); //wait for ACK
	mraa_gpio_write (pin1,HIGH);
	mraa_gpio_write (pin2,HIGH);
	mraa_gpio_dir(pin1, MRAA_GPIO_IN);
	while (mraa_gpio_read(pin1))
	{
		cnt0 += 1;
		if (cnt0 == 200)
		{
			mraa_gpio_dir(pin1, MRAA_GPIO_OUT);
			mraa_gpio_write(pin1, LOW);
			cnt0 = 0;
		}
	}
	//mraa_gpio_dir(pin1, MRAA_GPIO_OUT);
}
/**
 * \par Function
 *    start
 * \par Description
 *    Send start signal to TM1637
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \others
 *    None
 */	
void Me7SegmentDisplay::start(void)
{
	mraa_gpio_write (pin2,HIGH);
	mraa_gpio_write (pin1,HIGH);
	mraa_gpio_write (pin1,LOW);
	mraa_gpio_write (pin2,LOW);
}
/**
 * \par Function
 *    stop
 * \par Description
 *    Send the stop signal to TM1637.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \others
 *    None
 */
void Me7SegmentDisplay::stop(void)
{
	mraa_gpio_write (pin2,LOW);
	mraa_gpio_write (pin1,LOW);
	mraa_gpio_write (pin2,HIGH);
	mraa_gpio_write (pin1,HIGH);
}
/**
 * \par Function
 *    write
 * \par Description
 *    Write data to certain address.
 * \param[in]
 *    BitAddr - Bit address of data.
 * \param[in]
 *    SegData - Data to display.
 * \par Output
 *    None
 * \return
 *    None
 * \others
 *    None
 */
void Me7SegmentDisplay::write(uint8_t SegData[])
{
	uint8_t i;
	start();    // Start signal sent to TM1637 from MCU.
	writeByte(ADDR_AUTO);
	stop();
	start();
	writeByte(Cmd_SetAddr);
	for (i = 0; i < 4; i++)
	{
		writeByte(SegData[i]);
	}
	stop();
	start();
	writeByte(Cmd_DispCtrl);
	stop();
}
/**
 * \par Function
 *    display
 * \par Description
 *    Display 8 bit number array.
 * \param[in]
 *    DispData[] - The data that needs to be displayed store in this array.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */	
void Me7SegmentDisplay::display(uint8_t DispData[])
{
  uint8_t SegData[4];
  uint8_t i;
  for (i = 0; i < 4; i++)
  {
    SegData[i] = DispData[i];
  }
  coding(SegData);
  write(SegData);
}
/**
 * \par Function
 *    coding
 * \par Description
 *    Set display data using look up table.
 * \param[in]
 *    DispData[] - DataArray to display.
 * \par Output
 *    DispData[] - DataArray be transcoded.
 * \return
 *    None
 * \par Others
 *    None
 */	
void Me7SegmentDisplay::coding(uint8_t DispData[])
{
  for (uint8_t i = 0; i < 4; i++)
  {
    if (DispData[i] >= sizeof(tubeTable) / sizeof(*tubeTable))
    {
      DispData[i] = 32; // Change to ' '(space)
    }
    //DispData[i] = TubeTab[DispData[i]];
    DispData[i] = tubeTable[DispData[i]];//+ PointData;
  }
}
/**
 * \par Function
 *    display
 * \par Description
 *    Display certain value, and this value type is uint16_t
 * \param[in]
 *    value - Value to display.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void Me7SegmentDisplay::display(float value){
	uint8_t i=0;
	bool isStart = false;
	uint8_t index = 0;
	uint8_t disp[]={0,0,0,0};
	bool isNeg = false;
	if((float)value<0)
	{
		isNeg = true;
		value = -value;
		disp[0] = 0x21;
		index++;
	}
	for(i=0;i<7;i++)
	{
		int n = checkNum(value,3-i);
		if(n>=1||i==3)
		{
			isStart=true;
		}
		if(isStart)
		{
			if(i==3)
			{
				disp[index]=n+0x10;
			}
			else
			{
				disp[index]=n;
			}
			index++;
		}
		if(index>3)
		{
			break;
		}
	}
	display(disp);
}
/**
 * \Function
 *    clear
 * \Description
 *    Clear display.
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
void Me7SegmentDisplay::clear(void)
{
	uint8_t buf[4] = { ' ', ' ', ' ', ' ' };
	display(buf);
}
/**
 * \par Function
 *    checkNum
 * \par Description
 *    Extraction values to be displayed of float data
 * \param[in]
 *    v - Value to display.
 * \param[in]
 *    b - Value to display.
 * \par Output
 *    None
 * \return
 *    The data removal of the decimal point
 * \par Others
 *    None
 */
int16_t Me7SegmentDisplay::checkNum(float v,int16_t b)
{
	if(b>=0)
	{
		return floor((v-floor(v/pow(10,b+1))*(pow(10,b+1)))/pow(10,b));
	}
	else
	{
		b=-b;
		int i=0;
		for(i=0;i<b;i++)
		{
			v = v*10;
		}
		return ((int)(v)%10);
	}
}

/********************
*******MeShutter
*********************/	
/**
 * Alternate Constructor which can call your own function to map the MeShutter to arduino port,
 * and the shot and focus PIN will be set LOW
 * \param[in]
 *   port - RJ25 blue port(3,7,8) 
 */	
MeShutter::MeShutter(uint8_t port){
	mraa_init();
	MePort pt = MePort();
	uint8_t s1 = pt.getPin(port,0);
	uint8_t s2 = pt.getPin(port,1);
	pin1 = mraa_gpio_init(s1);
    if (pin1 == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s1);
        exit (1);
    }
	pin2 = mraa_gpio_init(s2);
    if (pin2 == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s2);
        exit (1);
    }
	mraa_gpio_use_mmaped(pin1, 1);
	mraa_gpio_use_mmaped(pin2, 1);
	mraa_gpio_dir(pin1, MRAA_GPIO_OUT);
	mraa_gpio_dir(pin2, MRAA_GPIO_OUT);
	mraa_gpio_write(pin1, LOW);
	mraa_gpio_write(pin2, LOW);
}
/**
 * Uninstall Constructor when you call function  ~MeShutter() 
 * \param[in]
 *   None
 */	
MeShutter::~MeShutter (){
	mraa_gpio_close(pin1);
	mraa_gpio_close(pin2);
}
/**
 * Uninstall Constructor when you call function  ~MeShutter() 
 * \param[in]
 *   None
 */
void MeShutter::shotOnTime(long time){
	mraa_gpio_write(pin1,HIGH);
	sleep(time);
	mraa_gpio_write(pin1,LOW);
}
/**
 * \par Function
 *   shotOn
 * \par Description
 *   Set the shot PIN on
 * \par Output
 *   None
 * \return
 *   None
 * \par Others
 *   None
 */
void MeShutter::shotOn(){
	mraa_gpio_write(pin1,HIGH);
}
/**
 * \par Function
 *   shotOff
 * \par Description
 *   Set the shot PIN off
 * \par Output
 *   None
 * \return
 *   None
 * \par Others
 *   None
 */
void MeShutter::shotOff(){
	mraa_gpio_write(pin1,LOW);
}
/**
 * \par Function
 *   focusOn
 * \par Description
 *   Set the focus PIN on
 * \par Output
 *   None
 * \return
 *   None
 * \par Others
 *   None
 */	
void MeShutter::focusOn(){
	mraa_gpio_write(pin2,HIGH);
}
/**
 * \par Function
 *   focusOff
 * \par Description
 *   Set the focus PIN off
 * \par Output
 *   None
 * \return
 *   None
 * \par Others
 *   None
 */
void MeShutter::focusOff(){
	mraa_gpio_write(pin2,LOW);
}
/********************
*******MeUltrasonicSensor
*********************/
/**
 * Alternate Constructor which can call your own function to map the ultrasonic Sensor to arduino port
 * \param[in]
 *   port - RJ25 yellow port 
 */
MeUltrasonicSensor::MeUltrasonicSensor(uint8_t port) {
    mraa_init();
	MePort pt = MePort();
	uint8_t s1 = pt.getPin(port,0);
	uint8_t s2 = pt.getPin(port,1);
	pin1 = mraa_gpio_init(s1);
	pin2 = mraa_gpio_init(s2);
    if (pin2 == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s2);
        exit (1);
    }
    m_doWork = 1;
	mraa_gpio_use_mmaped(pin2, 1);
    mraa_gpio_isr(pin2, MRAA_GPIO_EDGE_BOTH,&signalISR, this);
}
/**
 * Uninstall Constructor when you call function  ~MeUltrasonicSensor() 
 * \param[in]
 *   None
 */
MeUltrasonicSensor::~MeUltrasonicSensor () {
    mraa_gpio_close (pin1);
    mraa_gpio_close (pin2);
}
/**
 * \par Function
 *   measure
 * \par Description
 *   To get the duration of the ultrasonic sensor
 * \param[in]
 *   timeout - This value is used to define the measurement range, The
 *   default value is 30000.
 * \par Output
 *   None
 * \return
 *   The duration value associated with distance
 * \par Others
 *   None
 */
uint32_t MeUltrasonicSensor::measure(){
    mraa_gpio_dir(pin2, MRAA_GPIO_OUT);
	
    mraa_gpio_write(pin2, LOW);
    usleep(2);
    mraa_gpio_write(pin2, HIGH);
    usleep(5);
    mraa_gpio_write(pin2, LOW);
	
    m_doWork = 0;
    m_InterruptCounter = 0;
	
    int timer = 0;
    mraa_gpio_dir(pin2, MRAA_GPIO_IN);
	
    while (!m_doWork) {
		if(timer++>100){
			m_doWork = 1;
			return 0;
		}
        usleep (5);
    }
    return m_FallingTimeStamp - m_RisingTimeStamp;
}
/**
 * \par Function
 *   distanceCm
 * \par Description
 *   Centimeters return the distance
 * \param[in]
 *   MAXcm - The Max centimeters can be measured, the default value is 400.
 * \par Output
 *   None
 * \return
 *   The distance measurement in centimeters
 * \par Others
 *   None
 */
float MeUltrasonicSensor::distanceCm(){
	uint32_t dist = measure();
	return dist / 29.1;
}
/**
 * \par Function
 *   distanceInch
 * \par Description
 *   Inch return the distance
 * \param[in]
 *   MAXinch - The Max inch can be measured, the default value is 180.
 * \par Output
 *   None
 * \return
 *   The distance measurement in inch
 * \par Others
 *   None
 */	
float MeUltrasonicSensor::distanceInch(){
	uint32_t dist = measure();
	return dist / 74.1;
}
/**
 * \par Function
 *   distanceInch
 * \par Description
 *   return the m_doWork value
 * \param[in]
 *   None
 * \par Output
 *   None
 * \return
 *   m_doWork = 0
 * \par Others
 *   None
 */
bool MeUltrasonicSensor::isWorking(){
	return m_doWork==0;
}
void MeUltrasonicSensor::signalISR(void *ctx) {
    upm::MeUltrasonicSensor *This = (upm::MeUltrasonicSensor *)ctx;
    struct timeval timer;
    gettimeofday(&timer, NULL);

    This->m_InterruptCounter++;
    if (!(This->m_InterruptCounter % 2)) {
        This->m_FallingTimeStamp  = 1000000 * timer.tv_sec + timer.tv_usec;
        This->m_doWork = 1;
    } else {
        This->m_RisingTimeStamp = 1000000 * timer.tv_sec + timer.tv_usec;
    }
}
/***********************
*******Line Follower
************************/
/**
 * Alternate Constructor which can call your own function to map the line follwer device to arduino port,
 * no pins are used or initialized here.
 * \param[in]
 *   None
 */
MeLineFollower::MeLineFollower(uint8_t port) {
    mraa_init();
	MePort pt = MePort();
	uint8_t s1 = pt.getPin(port,0);
	uint8_t s2 = pt.getPin(port,1);
	pin1 = mraa_gpio_init(s1);
    if (pin1 == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s1);
        exit (1);
    }
	pin2 = mraa_gpio_init(s2);
    if (pin2 == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s2);
        exit (1);
    }
	mraa_gpio_use_mmaped(pin1, 1);
	mraa_gpio_use_mmaped(pin2, 1);
	mraa_gpio_dir(pin1, MRAA_GPIO_IN);
	mraa_gpio_dir(pin2, MRAA_GPIO_IN);
}
/**
 * Uninstall Constructor when you call function  ~MeLineFollower() 
 * \param[in]
 *   None
 */
MeLineFollower::~MeLineFollower () {
    mraa_gpio_close (pin1);
    mraa_gpio_close (pin2);
}
/**
 * \par Function
 *   read
 * \par Description
 *   Get the sensors state.
 * \par Output
 *   None
 * \return
 *   0: sensor1 is inside of black line \n
 *   1: sensor1 is outside of black line
 *   read function have 0,1,2,3 value
 * \par Others
 *   None
 */			
   
uint8_t MeLineFollower::read(){
	return (mraa_gpio_read(pin1)<<1)+mraa_gpio_read(pin2);
}
/***********************
*******Limit Switch
************************/
/**
 * Alternate Constructor which can call your own function to map the limit switch to arduino port,
 * no pins are used or initialized here.
 * \param[in]
 *   None
 */	
MeLimitSwitch::MeLimitSwitch(uint8_t port){
	mraa_init();
	MePort pt = MePort();
	uint8_t s1 = pt.getPin(port,0);
	uint8_t s2 = pt.getPin(port,1);
	pin1 = mraa_gpio_init(s1);
    if (pin1 == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s1);
        exit (1);
    }
	pin2 = mraa_gpio_init(s2);
    if (pin2 == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s2);
        exit (1);
    }
	mraa_gpio_use_mmaped(pin1, 1);
	mraa_gpio_use_mmaped(pin2, 1);
	mraa_gpio_mode(pin1,MRAA_GPIO_PULLUP);
	mraa_gpio_mode(pin2,MRAA_GPIO_PULLUP);
	mraa_gpio_dir(pin1, MRAA_GPIO_IN);
	mraa_gpio_dir(pin2, MRAA_GPIO_IN);
}
/**
 * Uninstall Constructor when you call function  ~MeLimitSwitch() 
 * \param[in]
 *   None
 */
MeLimitSwitch::~MeLimitSwitch (){
    mraa_gpio_close (pin1);
    mraa_gpio_close (pin2);
}
/**
 * \par Function
 *    read
 * \par Description
 *    Get switch value from selected _slot defined by MePort.
 * \param[in]
 *    None
 * \par Output
 *    slot - slot can used 0 and 1;
 * \Return
 *    True if module is touched.
 * \par Others
 *    None
 */
uint8_t MeLimitSwitch::read(uint8_t slot){
	return mraa_gpio_read(slot==0?pin1:pin2);
}
/***********************
*******MeTouchSensor
************************/
/**
 * Alternate Constructor which can call your own function to map the touch Sensor to arduino port
 * \param[in]
 *   port - RJ25 blue port
 */	
MeTouchSensor::MeTouchSensor(uint8_t port){
	
	mraa_init();
	MePort pt = MePort();
	uint8_t s1 = pt.getPin(port,0);
	uint8_t s2 = pt.getPin(port,1);
	pin1 = mraa_gpio_init(s1);
    if (pin1 == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s1);
        exit (1);
    }
	pin2 = mraa_gpio_init(s2);
    if (pin2 == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s2);
        exit (1);
    }
	mraa_gpio_use_mmaped(pin1, 1);
	mraa_gpio_use_mmaped(pin2, 1);
	mraa_gpio_dir(pin1, MRAA_GPIO_OUT);
	mraa_gpio_dir(pin2, MRAA_GPIO_IN);
}
/**
 * Uninstall Constructor when you call function  ~MeLimitSwitch() 
 * \param[in]
 *   None
 */
MeTouchSensor::~MeTouchSensor (){
    mraa_gpio_close (pin1);
    mraa_gpio_close (pin2);
}
/**
 * \par Function
 *    read
 * \par Description
 *    Get touch value from defined by MePort.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \Return
 *    True if module is touched.
 * \par Others
 *    None
 */
uint8_t MeTouchSensor::read(){
	return mraa_gpio_read(pin2);
}
/**
 * \par Function
 *   SetTogMode
 * \par Description
 *   Set the output type.
 * \param[in]
 *   TogMode - 1=> Toggle mode; 0(default)=>Direct mode
 * \par Output
 *   None
 * \return
 *   None
 * \par Others
 *   None
 */
void MeTouchSensor::setMode(uint8_t mode){
	mraa_gpio_write(pin1,mode);
}
/***********************
*******MePIRMotionSensor
************************/
/**
 * Alternate Constructor which can call your own function to map the Motion Sensor device to arduino port
 * \param[in]
 *   port - RJ25 blue port 
 */
MePIRMotionSensor::MePIRMotionSensor(uint8_t port){
	mraa_init();
	MePort pt = MePort();
	uint8_t s = pt.getPin(port,1);
	pin = mraa_gpio_init(s);
    if (pin == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s);
        exit (1);
    }
	mraa_gpio_use_mmaped(pin, 1);
	mraa_gpio_dir(pin, MRAA_GPIO_IN);
}
/**
 * Uninstall Constructor when you call function  ~MePIRMotionSensor() 
 * \param[in]
 *   None
 */
MePIRMotionSensor::~MePIRMotionSensor (){
	mraa_gpio_close(pin);
}
/**
 * \par Function
 *    read
 * \par Description
 *    Get PIRMotionSensor value 
 *    None
 * \par Output
 *    None
 * \Return
 *    True if module is touched.
 * \par Others
 *    None
 */
uint8_t MePIRMotionSensor::read(){
	return mraa_gpio_read(pin);
}
/***********************
*******Light Sensor
************************/
/**
 * Alternate Constructor which can call your own function to map the light sensor to arduino port
 * \param[in]
 *   port - RJ25 black port
 */
MeLightSensor::MeLightSensor(uint8_t port) {
    mraa_init();
	MePort pt = MePort();
	uint8_t s2 = pt.getPin(port,1);
	pin = mraa_aio_init(s2-14);
    if (pin == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s2);
        exit (1);
    }
}
/**
 * Uninstall Constructor when you call function  ~MeLightSensor() 
 * \param[in]
 *   None
 */
MeLightSensor::~MeLightSensor () {
    mraa_aio_close(pin);
}
/**
 * \par Function
 *    read
 * \par Description
 *    Get LightSensor value 
 *    None
 * \par Output
 *    None
 * \Return
 *    True if module is touched.
 * \par Others
 *    None
 */
uint16_t MeLightSensor::read(){
	return mraa_aio_read(pin);
}
/***********************
*******Me 4Button
************************/
/**
 *  Alternate Constructor which can call your own function to map the Me4Button to arduino port, \n
 * \param[in]
 *    port - RJ25 black port
 */
Me4Button::Me4Button(uint8_t port) {
    mraa_init();
	MePort pt = MePort();
	uint8_t s2 = pt.getPin(port,1);
	pin = mraa_aio_init(s2-14);
    if (pin == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s2);
        exit (1);
    }
}
 /**
 * Uninstall Constructor when you call function  ~Me4Button() 
 * \param[in]
 *   None
 */
Me4Button::~Me4Button () {
    mraa_aio_close(pin);
}
/**
 * \par Function
 *    read
 * \par Description
 *    Get 4Button value 
 *    None
 * \par Output
 *    None
 * \Return
 *    Button value
 * \par Others
 *    None
 */
uint8_t Me4Button::read(){
	uint16_t v = mraa_aio_read(pin);
	return v<100?1:(v<500?2:(v<700?3:(v<800?4:0)));
}
/***********************
*******MeSoundSensor
************************/
/**
 * Alternate Constructor which can call your own function to map the Sound Sensor to arduino port
 * \param[in]
 *   port - RJ25 black port
 */
MeSoundSensor::MeSoundSensor(uint8_t port){
	mraa_init();
	MePort pt = MePort();
	uint8_t s2 = pt.getPin(port,1);
	pin = mraa_aio_init(s2-14);
    if (pin == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s2);
        exit (1);
    }
}
/**
 * Uninstall Constructor when you call function  ~MeSoundSensor() 
 * \param[in]
 *   None
 */
MeSoundSensor::~MeSoundSensor(){
	mraa_aio_close(pin);
}
/**
 * \par Function
 *    read
 * \par Description
 *    Get SoundSensor value 
 *    None
 * \par Output
 *    None
 * \Return
 *    Sound sensor value
 * \par Others
 *    None
 */
uint16_t MeSoundSensor::read(){
	return mraa_aio_read(pin);
}
/***********************
*******MePotentiometer
************************/
/**
 * Alternate Constructor which can call your own function to map the Me potentiometer device to arduino port
 * \param[in]
 *   port - RJ25 black port
 */
MePotentiometer::MePotentiometer(uint8_t port){
	mraa_init();
	MePort pt = MePort();
	uint8_t s2 = pt.getPin(port,1);
	pin = mraa_aio_init(s2-14);
    if (pin == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s2);
        exit (1);
    }
}
/**
 * Uninstall Constructor when you call function  ~MePotentiometer() 
 * \param[in]
 *   None
 */
MePotentiometer::~MePotentiometer(){
	mraa_aio_close(pin);
}
/**
 * \par Function
 *    read
 * \par Description
 *    Get Potentiometer value 
 *    None
 * \par Output
 *    None
 * \Return
 *    Potentiometer value
 * \par Others
 *    None
 */
uint16_t MePotentiometer::read(){
	return mraa_aio_read(pin);
}
/***********************
*******MeJoystick
************************/
/**
 * Alternate Constructor which can call your own function to map the MeJoystick to arduino port.
 * \param[in]
 *   port - RJ25 black port
 */
MeJoystick::MeJoystick(uint8_t port){
	mraa_init();
	MePort pt = MePort();
	uint8_t s1 = pt.getPin(port,0);
	uint8_t s2 = pt.getPin(port,1);
	pin1 = mraa_aio_init(s1-14);
	pin2 = mraa_aio_init(s2-14);
    if (pin1 == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s1);
        exit (1);
    }
    if (pin2 == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s2);
        exit (1);
    }
}
/**
 * Uninstall Constructor when you call function  ~MeJoystick() 
 * \param[in]
 *   None
 */	
MeJoystick::~MeJoystick(){
	mraa_aio_close(pin1);
	mraa_aio_close(pin2);
}
/**
 * \par Function
 *    read
 * \par Description
 *    Get joystick value 
 *    None
  * \param[in]
 *    axis - 0 is x value and 1 is y value
 * \par Output
 *    None
 * \Return
 *    Potentiometer value
 * \par Others
 *    None
 */
uint16_t MeJoystick::read(uint8_t axis){
	return mraa_aio_read(axis==0?pin1:pin2);
}
/***********************
*******MeHumiture
************************/
/**
 * Alternate Constructor which can call your own function to map the flame snesor to arduino port
 * \param[in]
 *   port - RJ25 black port
 */
MeFlameSensor::MeFlameSensor(uint8_t port){
	mraa_init();
	MePort pt = MePort();
	uint8_t s1 = pt.getPin(port,0);
	uint8_t s2 = pt.getPin(port,1);
	pin1 = mraa_aio_init(s1-14);
	pin2 = mraa_gpio_init(s2);
    if (pin1 == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s1);
        exit (1);
    }
    if (pin2 == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s2);
        exit (1);
    }
	mraa_gpio_use_mmaped(pin2, 1);
	mraa_gpio_dir(pin2, MRAA_GPIO_IN);
}
/**
 * Uninstall Constructor when you call function  ~MeFlameSensor() 
 * \param[in]
 *   None
 */
MeFlameSensor::~MeFlameSensor(){
	mraa_aio_close(pin1);
	mraa_gpio_close(pin2);
}
/**
 * \par Function
 *    read
 * \par Description
 *    Get flame sensor value 
 *    None
  * \param[in]
 *   mode - 0 or 1
 * \par Output
 *    None
 * \Return
 *    Flame sensor value
 * \par Others
 *    None
 */
uint16_t MeFlameSensor::read(uint8_t mode){
	if(mode==0){
		return mraa_aio_read(pin1);
	}else{
		return mraa_gpio_read(pin2);
	}
}
/***********************
*******MeGasSensor
************************/
/**
 * Alternate Constructor which can call your own function to map the gas snesor to arduino port
 * \param[in]
 *   port - RJ25 black port
 */	
MeGasSensor::MeGasSensor(uint8_t port){
	mraa_init();
	MePort pt = MePort();
	uint8_t s1 = pt.getPin(port,0);
	uint8_t s2 = pt.getPin(port,1);
	pin1 = mraa_aio_init(s1-14);
	pin2 = mraa_gpio_init(s2);
    if (pin1 == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s1);
        exit (1);
    }
    if (pin2 == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s2);
        exit (1);
    }
	mraa_gpio_use_mmaped(pin2, 1);
	mraa_gpio_dir(pin2, MRAA_GPIO_IN);
}
/**
 * Uninstall Constructor when you call function  ~MeGasSensor() 
 * \param[in]
 *   None
 */
MeGasSensor::~MeGasSensor(){
	mraa_aio_close(pin1);
	mraa_gpio_close(pin2);
}
/**
 * \par Function
 *    read
 * \par Description
 *    Get gas sensor value 
 *    None
  * \param[in]
 *   mode - 0 or 1
 * \par Output
 *    None
 * \Return
 *    Gas sensor value
 * \par Others
 *    None
 */
uint16_t MeGasSensor::read(uint8_t mode){
	if(mode==0){
		return mraa_aio_read(pin1);
	}else{
		return mraa_gpio_read(pin2);
	}
}
/***********************
*******MeHumiture
************************/
/**
 * Alternate Constructor which can call your own function to map the humiture sensor to arduino port,
 * the slot2 pin will be used here since specify slot is not be set.
 * \param[in]
 *   port - RJ25 yellow port
 */
MeHumiture::MeHumiture(uint8_t port){
	mraa_init();
	MePort pt = MePort();
	uint8_t s2 = pt.getPin(port,1);
	pin2 = mraa_gpio_init(s2);
    if (pin2 == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s2);
        exit (1);
    }
	mraa_gpio_use_mmaped(pin2, 1);
}
/**
 * Uninstall Constructor when you call function  ~MeHumiture() 
 * \param[in]
 *   None
 */
MeHumiture::~MeHumiture(){
    mraa_gpio_close (pin2);
}
/**
 * \par Function
 *   update
 * \par Description
 *   Use this function to update the sensor data
 * \par Output
 *   None
 * \return
 *   None
 * \par Others
 *   None
 */
int MeHumiture::update(){
	uint8_t data[5] = {0};
	
	uint8_t cnt = 7;
	uint8_t idx = 0;
	uint8_t time = 0;
	
	// REQUEST SAMPLE
	mraa_gpio_dir(pin2, MRAA_GPIO_OUT);
	mraa_gpio_write(pin2, LOW);
	usleep(18000);
	mraa_gpio_write(pin2, HIGH);
	usleep(40);
	mraa_gpio_dir(pin2, MRAA_GPIO_IN);
	mraa_gpio_write(pin2, LOW);
	
	// ACKNOWLEDGE or TIMEOUT
	//unsigned int loopCnt = 1000;
	while(mraa_gpio_read(pin2) == LOW){
		//if (loopCnt-- == 0) 
		//	return DHTLIB_ERROR_TIMEOUT;
	}
	
	//loopCnt = 100000;
	while(mraa_gpio_read(pin2) == HIGH){
		//if (loopCnt-- == 0) 
		//	return DHTLIB_ERROR_TIMEOUT;
	}

	// READ OUTPUT - 40 BITS => 5 BYTES or TIMEOUT
	for(int16_t i=0;i<40;i++){
		//loopCnt = 100000;
		while(mraa_gpio_read(pin2) == LOW){
			//if (loopCnt-- == 0) 
			//	return DHTLIB_ERROR_TIMEOUT;
		}
		
		//loopCnt = 100000;
		while(mraa_gpio_read(pin2) == HIGH){
			//if (loopCnt-- == 0) 
			//	return DHTLIB_ERROR_TIMEOUT;
			time ++;
		}
		
		if (time > 40) data[idx] |= (1 << cnt);
		time = 0;
		if (cnt == 0){	// next byte
			cnt = 7;    // restart at MSB
			idx++;      // next byte!
		}
		else cnt--;
	}
	
	humidity = data[0];
	temperature = data[2];
	
	uint8_t sum = data[0] + data[2];
	
	if(data[4] != sum) 
		return DHTLIB_ERROR_CHECKSUM;
	
	return DHTLIB_OK;

}
void MeHumiture::signalISR(void *ctx) {
    upm::MeHumiture *This = (upm::MeHumiture *)ctx;
    struct timeval timer;
    gettimeofday(&timer, NULL);

    This->m_InterruptCounter++;
    if (!(This->m_InterruptCounter % 2)) {
        This->m_FallingTimeStamp  = 1000000 * timer.tv_sec + timer.tv_usec;
        This->m_doWork = 1;
    } else {
        This->m_RisingTimeStamp = 1000000 * timer.tv_sec + timer.tv_usec;
    }
}
/**
 * \par Function
 *   getHumidity
 * \par Description
 *   Use this function to Get the Humidity data
 * \par Output
 *   None
 * \return
 *   The value of Humidity
 * \par Others
 *   None
 */
uint8_t MeHumiture::getHumidity(){
	return humidity;
}
/**
 * \par Function
 *   getTemperature
 * \par Description
 *   Use this function to Get the Temperature data
 * \par Output
 *   None
 * \return
 *   The value of Temperature
 * \par Others
 *   None
 */
uint8_t MeHumiture::getTemperature(){
	return temperature;
}
/**
 * \par Function
 *    Kelvin
 * \par Description
 *    Change celsius degrees into Kelvin.
 * \param[in]
 *    celsius - The number of celsius degrees.
 * \par Output
 *    None
 * \par Return
 *    Return the number of Kelvin temperature.
 * \par Others
 *    None
 */
double MeHumiture::getKelvin(void)
{
  return temperature + 273.15;
}
/**
 * \par Function
 *    Fahrenheit
 * \par Description
 *    Change celsius degrees into Fahrenheit.
 * \param[in]
 *    celsius - The number of celsius degrees.
 * \par Output
 *    None
 * \par Return
 *    Return the number of Fahrenheit
 * \par Others
 *    None
 */
double MeHumiture::getFahrenheit(){
	return 1.8 * temperature + 32;
}
/**
 * \par Function
 *    dewPoint
 * \par Description
 *    The dew-point temperature (Point at this temperature, the air is saturated and produce dew).
 * \param[in]
 *    celsius - The celsius degrees of air.
  * \param[in]
 *    humidity - The humidity of air.
 * \par Output
 *    None
 * \par Return
 *    Return the dew-point of air.
 * \par Others
 *    None
 */
double MeHumiture::getDewPoint(void)
{
  double a0= 373.15/(273.15 + temperature);
  double sum = -7.90298 * (a0-1);
  sum += 5.02808 * log10(a0);
  sum += -1.3816e-7 * (pow(10, (11.344*(1-1/a0)))-1) ;
  sum += 8.1328e-3 * (pow(10,(-3.49149*(a0-1)))-1) ;
  sum += log10(1013.246);
  double vp = pow(10, sum-3) * humidity;
  double t = log(vp/0.61078);   // temp var
  return (241.88 * t) / (17.558-t);
}
/**
 * \par Function
 *    dewPointFast
 * \par Description
 *    Fast calculating dew point, Speed is 5 times to getDewPoint().
 * \param[in]
 *    celsius - The celsius degrees of air.
  * \param[in]
 *    humidity - The humidity of air.
 * \par Output
 *    None
 * \par Return
 *    Return the Fast calculating dew point of air.
 * \par Others
 *    None
 */	
double MeHumiture::getPointFast()
{
  double a = 17.271;
  double b = 237.7;
  double temp = (a * temperature) / (b + temperature) + log(humidity/100);
  //double Td = (b * temp) / (a - temp);
  return ((b * temp) / (a - temp));
}
/***********************
*******MeGyro
************************/
/**
 * Alternate Constructor which can call your own function to map the MeGyro to arduino port,
 * no pins are used or initialized here
 */
MeGyro::MeGyro(){
}
/**
 * Uninstall Constructor when you call function  ~MeGyro() 
 * \param[in]
 *   None
 */
MeGyro::~MeGyro(){
	
}
/**
 * \par Function
 *   begin
 * \par Description
 *   Initialize the MeGyro.
 * \param[in]
 *   None
 * \par Output
 *   None
 * \return
 *   None
 * \par Others
 *   You can check the MPU6050 datasheet for the registor address.
 */
void MeGyro::begin(){
	mpu6050.init();
}
/**
 * \par Function
 *   update
 * \par Description
 *   Update some calculated angle values to the variable.
 * \param[in]
 *   None
 * \par Output
 *   None
 * \return
 *   None
 * \par Others
 *   The angle values are calculated by complementary filter.
 *   The time constant of filter is set to 0.5 second, but period dt is not a constant, 
 *   so the filter coefficient will be calculated dynamically.
 */	
void MeGyro::update(void)
{
	mpu6050.update();
	mpu6050.getGyroscope(&gx,&gy,&gz);
}
/**
 * \par Function
 *   read
 * \par Description
 *   Get the angle value of x-axis or y-axis,or z-axis.
 * \param[in]
 *   axis - 0 = x-axis , 1 = y-axis,2 = z-axis
 * \par Output
 *   None
 * \return
 *   The angle value of x or y or z axis
 * \par Others
 *   X-axis angle value is calculated by complementary filter.
 */
double MeGyro::read(uint8_t axis){
	return axis==0?gx:(axis==1?gy:gz);
}
/***********************
*******MeTftLCD
************************/
/**
 * Alternate Constructor which can call your own function to map the TftLCD to arduino port,
 * no pins are used or initialized here
 */
MeTftLCD::MeTftLCD(){
	uart = mraa_uart_init(0);
	mraa_uart_set_baudrate (uart, 9600);
}
/**
 * Uninstall Constructor when you call function  ~MeTftLCD() 
 * \param[in]
 *   None
 */
MeTftLCD::~MeTftLCD(){
    mraa_uart_stop(uart);
}
/**
 * \par Function
 *   send
 * \par Description
 *   Send the string command to TFT LCD Screen
 * \param[in]
 *   str - used sting
 * \par Output
 *   None
 * \return
 *   None
 * \par Others
 *  None
 */
void MeTftLCD::send(char *str){
	mraa_uart_write(uart, str, strlen(str));
}
	
MeRGBLed::MeRGBLed(uint8_t port){
	mraa_init();
	MePort pt = MePort();
	uint8_t s2 = pt.getPin(port,1);
	pin1 = mraa_gpio_init(s2);
    if (pin1 == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s2);
        exit (1);
    }
	mraa_gpio_use_mmaped (pin1,1);
	mraa_gpio_dir (pin1, MRAA_GPIO_OUT);
	setNumber(32);
}
 
MeRGBLed::~MeRGBLed(){
	mraa_gpio_close (pin1);
}

void MeRGBLed::setNumber(uint8_t num_leds)
{
  count_led = num_leds;
  pixels    = (uint8_t*)malloc(count_led * 3);
  if(!pixels)
  {
    printf("There is not enough space!\r\n");
  }
  for(int16_t i = 0; i < count_led * 3; i++)
  {
    pixels[i] = 0;
  }
}

bool MeRGBLed::setColorAt(uint8_t index, uint8_t red, uint8_t green, uint8_t blue){
	if(index < count_led)
    {
		uint8_t tmp = index * 3;
		pixels[tmp] = green;
		pixels[tmp + 1] = red;
		pixels[tmp + 2] = blue;
		return(true);
    }
    return(false);
}
