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