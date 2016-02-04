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

MePort::MePort(){
	 uint8_t list[30] = {NC, NC , 11, 10 ,  9, 12 , 13,  8 , NC,  3 , NC, NC , NC,  2 , A2, A3 , A0, A1 ,  5,  4 ,  6,  7 , NC, NC , NC, NC , NC, NC , NC, NC};
	 ports = (uint8_t*)malloc(30);
	 for(int i=0;i<30;i++)ports[i] = list[i];
}
uint8_t MePort::getPin(uint8_t port,uint8_t slot){
	return ports[port*2+slot];
}
/********************
*******Me DC Motor
*********************/
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
	mraa_gpio_use_mmaped(pin2, 1);
	mraa_gpio_dir(pin2, MRAA_GPIO_OUT);
}
MeDCMotor::~MeDCMotor (){
    mraa_pwm_close(pin1);
    mraa_gpio_close(pin2);
}
void MeDCMotor::run(int16_t pwm){
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
MeServoMotor::MeServoMotor(uint8_t port,uint8_t slot){
	MePort pt = MePort();
	int s = pt.getPin(port,slot);
    svr = new Servo(s);
	
}
MeServoMotor::~MeServoMotor (){
}
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
MeStepperMotor::~MeStepperMotor (){
    mraa_gpio_close (pin1);
    mraa_gpio_close (pin2);
}
void MeStepperMotor::step(){
	mraa_gpio_write(pin1, _dir==DIRECTION_CW?LOW:HIGH);
	mraa_gpio_write(pin2,HIGH);
	usleep(1);
	mraa_gpio_write(pin2,LOW);
}
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
unsigned long MeStepperMotor::ctime(){
	gettimeofday(&timer, NULL);
	return 1000000 * timer.tv_sec + timer.tv_usec;
}
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
long MeStepperMotor::currentPosition(){
	return _currentPos;
}
long MeStepperMotor::distanceToGo()
{
	return _targetPos - _currentPos;
}
unsigned long MeStepperMotor::stepInterval(){
	return _stepInterval;
}
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
void MeStepperMotor::move(long distance){
	 moveTo(_currentPos + distance);
}
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
Me7SegmentDisplay::~Me7SegmentDisplay (){
    mraa_gpio_close (pin1);
    mraa_gpio_close (pin2);
}
void Me7SegmentDisplay::set(uint8_t brightness, uint8_t SetData, uint8_t SetAddr)
{
  Cmd_SetData = SetData;
  Cmd_SetAddr = SetAddr;
  Cmd_DispCtrl = SEGDIS_ON + brightness;//Set brightness, take effect next display cycle.
}
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
	mraa_gpio_dir(pin1, MRAA_GPIO_OUT);
}

void Me7SegmentDisplay::start(void)
{
	mraa_gpio_write (pin2,HIGH);
	mraa_gpio_write (pin1,HIGH);
	mraa_gpio_write (pin1,LOW);
	mraa_gpio_write (pin2,LOW);
}
void Me7SegmentDisplay::stop(void)
{
	mraa_gpio_write (pin2,LOW);
	mraa_gpio_write (pin1,LOW);
	mraa_gpio_write (pin2,HIGH);
	mraa_gpio_write (pin1,HIGH);
}
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
void Me7SegmentDisplay::clear(void)
{
	uint8_t buf[4] = { ' ', ' ', ' ', ' ' };
	display(buf);
}
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
MeShutter::~MeShutter (){
	mraa_gpio_close(pin1);
	mraa_gpio_close(pin2);
}
void MeShutter::shotOnTime(long time){
	mraa_gpio_write(pin1,HIGH);
	sleep(time);
	mraa_gpio_write(pin1,LOW);
}
void MeShutter::shotOn(){
	mraa_gpio_write(pin1,HIGH);
}
void MeShutter::shotOff(){
	mraa_gpio_write(pin1,LOW);
}
void MeShutter::focusOn(){
	mraa_gpio_write(pin2,HIGH);
}
void MeShutter::focusOff(){
	mraa_gpio_write(pin2,LOW);
}
/********************
*******MeUltrasonicSensor
*********************/
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

MeUltrasonicSensor::~MeUltrasonicSensor () {
    mraa_gpio_close (pin1);
    mraa_gpio_close (pin2);
}
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
float MeUltrasonicSensor::distanceCm(){
	uint32_t dist = measure();
	return dist / 29.1;
}
float MeUltrasonicSensor::distanceInch(){
	uint32_t dist = measure();
	return dist / 74.1;
}
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

MeLineFollower::~MeLineFollower () {
    mraa_gpio_close (pin1);
    mraa_gpio_close (pin2);
}
uint8_t MeLineFollower::read(){
	return (mraa_gpio_read(pin1)<<1)+mraa_gpio_read(pin2);
}
/***********************
*******Limit Switch
************************/
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
MeLimitSwitch::~MeLimitSwitch (){
    mraa_gpio_close (pin1);
    mraa_gpio_close (pin2);
}
uint8_t MeLimitSwitch::read(uint8_t slot){
	return mraa_gpio_read(slot==0?pin1:pin2);
}
/***********************
*******MeTouchSensor
************************/
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
MeTouchSensor::~MeTouchSensor (){
    mraa_gpio_close (pin1);
    mraa_gpio_close (pin2);
}
uint8_t MeTouchSensor::read(){
	return mraa_gpio_read(pin2);
}
void MeTouchSensor::setMode(uint8_t mode){
	mraa_gpio_write(pin1,mode);
}
/***********************
*******MePIRMotionSensor
************************/
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
MePIRMotionSensor::~MePIRMotionSensor (){
	mraa_gpio_close(pin);
}
uint8_t MePIRMotionSensor::read(){
	return mraa_gpio_read(pin);
}
/***********************
*******Light Sensor
************************/
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

MeLightSensor::~MeLightSensor () {
    mraa_aio_close(pin);
}
uint16_t MeLightSensor::read(){
	return mraa_aio_read(pin);
}
/***********************
*******Me 4Button
************************/
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

Me4Button::~Me4Button () {
    mraa_aio_close(pin);
}
uint8_t Me4Button::read(){
	uint16_t v = mraa_aio_read(pin);
	return v<100?1:(v<500?2:(v<700?3:(v<800?4:0)));
}
/***********************
*******MeSoundSensor
************************/
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
MeSoundSensor::~MeSoundSensor(){
	mraa_aio_close(pin);
}
uint16_t MeSoundSensor::read(){
	return mraa_aio_read(pin);
}
/***********************
*******MePotentiometer
************************/
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
MePotentiometer::~MePotentiometer(){
	mraa_aio_close(pin);
}
uint16_t MePotentiometer::read(){
	return mraa_aio_read(pin);
}
/***********************
*******MeJoystick
************************/
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
MeJoystick::~MeJoystick(){
	mraa_aio_close(pin1);
	mraa_aio_close(pin2);
}
uint16_t MeJoystick::read(uint8_t axis){
	return mraa_aio_read(axis==0?pin1:pin2);
}
/***********************
*******MeHumiture
************************/
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
MeFlameSensor::~MeFlameSensor(){
	mraa_aio_close(pin1);
	mraa_gpio_close(pin2);
}
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
MeGasSensor::~MeGasSensor(){
	mraa_aio_close(pin1);
	mraa_gpio_close(pin2);
}
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
MeHumiture::MeHumiture(uint8_t port){
	 mraa_init();
	MePort pt = MePort();
	//uint8_t s1 = pt.getPin(port,0);
	uint8_t s2 = pt.getPin(port,1);
	//pin1 = mraa_gpio_init(s1);
    //if (pin1 == NULL) {
     //   fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s1);
    //    exit (1);
    //}
	pin2 = mraa_gpio_init(s2);
    if (pin2 == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", s2);
        exit (1);
    }
	//mraa_gpio_use_mmaped(pin1, 1);
	mraa_gpio_use_mmaped(pin2, 1);
	//mraa_gpio_dir(pin1, MRAA_GPIO_IN);
	//mraa_gpio_dir(pin2, MRAA_GPIO_IN);
}
MeHumiture::~MeHumiture(){
    //mraa_gpio_close (pin1);
    mraa_gpio_close (pin2);
}
void MeHumiture::update(){
	uint8_t data[5] = {0};
	uint16_t i;
	mraa_gpio_dir(pin2, MRAA_GPIO_OUT);
	for (i=0;i<10000;i++){
		mraa_gpio_write(pin2, LOW);
	}
	for (i=0;i<10;i++){
		mraa_gpio_write(pin2, HIGH);
	}
	for (i=0;i<10;i++){
		mraa_gpio_write(pin2, LOW);
	}
	mraa_gpio_dir(pin2, MRAA_GPIO_IN);
	uint16_t time = 0;
	while(mraa_gpio_read(pin2) != HIGH){
		time++;
		if( time > 2000)
		{
		  humidity = 0;
		  temperature = 0;
		  break;
		}
		usleep(1);
	}
	time = 0;
	while(mraa_gpio_read(pin2) != LOW){
		time++;
		if(  time  > 2000)
		{
			break;
		}
		usleep(1);
	}

	for(int16_t i=0;i<40;i++){
		time = 0;
		while(mraa_gpio_read(pin2) == LOW)
		{
			time++;
			if( time  > 2000)
			{
				break;
			}
			usleep(1);
		}
		time = 0;
		while(mraa_gpio_read(pin2) == HIGH)
		{
			time++;
			if( time  > 2000 )
			{
				break;
			}
			usleep(1);
		}
		if ( time > 4 )
		{
		  data[i/8] <<= 1;
		  data[i/8] |= 0x01;
		}
		else
		{
		  data[i/8] <<= 1;
		}
	}
	if( (data[0] + data[2]) == data[4] )
	{
		humidity = data[0];
		temperature = data[2];
	}
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
uint8_t MeHumiture::getHumidity(){
	return humidity;
}
uint8_t MeHumiture::getTemperature(){
	return temperature;
}
double MeHumiture::getKelvin(void)
{
  return temperature + 273.15;
}
double MeHumiture::getFahrenheit(){
	return 1.8 * temperature + 32;
}
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
MeGyro::MeGyro(){
}
MeGyro::~MeGyro(){
	
}
void MeGyro::begin(){
	mpu6050.init();
}
void MeGyro::update(void)
{
	mpu6050.update();
	mpu6050.getGyroscope(&gx,&gy,&gz);
}
double MeGyro::read(uint8_t axis){
	return axis==0?gx:(axis==1?gy:gz);
}
/***********************
*******MeTftLCD
************************/
MeTftLCD::MeTftLCD(){
	uart = mraa_uart_init(0);
	mraa_uart_set_baudrate (uart, 9600);
}
MeTftLCD::~MeTftLCD(){
    mraa_uart_stop(uart);
}
void MeTftLCD::send(char *str){
	mraa_uart_write(uart, str, strlen(str));
}