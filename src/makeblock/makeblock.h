#pragma once
 
#include <stdio.h>
#include <mraa/aio.h>
#include <mraa/gpio.h>

#define HIGH                   1
#define LOW                    0

namespace upm {
 
    class MyModule {
        public:
 
            MyModule();
			~MyModule ();
            void open(uint8_t pin);
			void lowCode();
			void highCode();
			void reset();
			void setColorAt(uint8_t index,uint8_t r,uint8_t g,uint8_t b);
			void show();
			void setCount(uint8_t len);
			void sendPixels();
        private:
			void setColor(uint8_t r,uint8_t g,uint8_t b);
			uint8_t *buffer;
			uint8_t maxCount;
			mraa_gpio_context gpio;
    };
}
