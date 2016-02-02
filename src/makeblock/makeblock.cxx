#include "mymodule.h"
 
#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <functional>

using namespace upm;

MyModule::MyModule() {
    mraa_init();
	buffer = (uint8_t*)malloc(12);
}
 
MyModule::~MyModule () {
	free(buffer);
    mraa_gpio_close (gpio);
}
void MyModule::open(uint8_t pin) {
    // setup pin
    gpio = mraa_gpio_init(pin);
    if (gpio == NULL) {
        fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", 13);
        exit (1);
    }
	mraa_gpio_use_mmaped(gpio, 1);
	setCount(16);
	show();
}
void MyModule::lowCode()
{
	//---- T0H ----
	mraa_gpio_write(gpio , 1);
	//---- T0L ----
	mraa_gpio_write(gpio , 0);
	mraa_gpio_write(gpio , 0);
	mraa_gpio_write(gpio , 0);
	mraa_gpio_write(gpio , 0);
}
void MyModule::highCode()
{
	//---- T1H ----
	mraa_gpio_write(gpio , 1);
	mraa_gpio_write(gpio , 1);
	mraa_gpio_write(gpio , 1);
	//---- T1L ----
	mraa_gpio_write(gpio , 0);
	mraa_gpio_write(gpio , 0);
	
}
void MyModule::reset()
{
	int i=0;
	for(i=0;i<100;i++) {
		mraa_gpio_write(gpio , 0);
		mraa_gpio_write(gpio , 0);
		mraa_gpio_write(gpio , 0);
		mraa_gpio_write(gpio , 0);
		mraa_gpio_write(gpio , 0);
	}
}

void MyModule::setColorAt(uint8_t index,uint8_t r,uint8_t g,uint8_t b){
	if(index<maxCount){
		buffer[index*3] = r;
		buffer[index*3+1] = g;
		buffer[index*3+2] = b;
	}
}
void MyModule::show(){
	uint8_t i = 0;
	//reset();
	for(i=0;i<maxCount;i++){
		setColor(buffer[i*3],buffer[i*3+1],buffer[i*3+2]);
	}
}
void MyModule::setCount(uint8_t len){
	maxCount = len;
	free(buffer);
	buffer = (uint8_t*)malloc(len*3);
	int i = 0;
	for(i=0;i<maxCount;i++){
		setColorAt(i,0,0,0);
	}
}
void MyModule::sendPixels()
{
    uint8_t mask;
    uint8_t subpix;
    uint32_t cyclesStart;

    // trigger emediately
	int count = 0;
	int i = 0;
	uint8_t cyclesBit;
	uint8_t total = maxCount*3;
    while (count < total)
    {
		subpix = buffer[count];
		count++;
        for(i=0;i<8;i++){
            cyclesBit = (subpix & (0x80>>i));
			if(cyclesBit==0){
				mraa_gpio_write(gpio , 1);
				mraa_gpio_write(gpio , 0);
				mraa_gpio_write(gpio , 0);
				mraa_gpio_write(gpio , 0);
			}else{
				mraa_gpio_write(gpio , 1);
				mraa_gpio_write(gpio , 1);
				mraa_gpio_write(gpio , 1);
				mraa_gpio_write(gpio , 0);
			}
        }
    }
    // while accurate, this isn't needed due to the delays at the 
    // top of Show() to enforce between update timing
    // while ((_getCycleCount() - cyclesStart) < CYCLES_400);
}
void MyModule::setColor(uint8_t r,uint8_t g,uint8_t b)
{
	int i=0;
	for(i=0;i<8;i++){
		if( (g & (0x80>>i)) ==0) {
			mraa_gpio_write(gpio , 1);
			mraa_gpio_write(gpio , 0);
			mraa_gpio_write(gpio , 0);
			mraa_gpio_write(gpio , 0);
			mraa_gpio_write(gpio , 0);
		}else {		
			mraa_gpio_write(gpio , 1);
			mraa_gpio_write(gpio , 1);
			mraa_gpio_write(gpio , 1);
			mraa_gpio_write(gpio , 0);
			mraa_gpio_write(gpio , 0);
		}
	}
	for(i=0;i<8;i++){
		if( (r & (0x80>>i)) ==0) {
			mraa_gpio_write(gpio , 1);
			mraa_gpio_write(gpio , 0);
			mraa_gpio_write(gpio , 0);
			mraa_gpio_write(gpio , 0);
			mraa_gpio_write(gpio , 0);
		}else {		
			mraa_gpio_write(gpio , 1);
			mraa_gpio_write(gpio , 1);
			mraa_gpio_write(gpio , 1);
			mraa_gpio_write(gpio , 0);
			mraa_gpio_write(gpio , 0);
		}
	}
	for(i=0;i<8;i++){
		if( (b & (0x80>>i)) ==0){
			mraa_gpio_write(gpio , 1);
			mraa_gpio_write(gpio , 0);
			mraa_gpio_write(gpio , 0);
			mraa_gpio_write(gpio , 0);
			mraa_gpio_write(gpio , 0);
		}else {		
			mraa_gpio_write(gpio , 1);
			mraa_gpio_write(gpio , 1);
			mraa_gpio_write(gpio , 1);
			mraa_gpio_write(gpio , 0);
			mraa_gpio_write(gpio , 0);
		}
	}
}