#pragma once
 
#include <stdio.h>
#include <mraa/aio.h>
#include <mraa/gpio.h>
#include <sys/time.h>

#define HIGH                   1
#define LOW                    0
#define PORT_1 1
#define PORT_2 2
#define PORT_3 3
#define PORT_4 4
#define PORT_5 5
#define PORT_6 6
#define PORT_7 7
#define PORT_8 8
#define PORT_9 9
#define PORT_10 10




namespace upm {
	class MePort{
		public:
			MePort();
			uint8_t getPin(uint8_t port,uint8_t slot);
		private:
			uint8_t *ports;
	};
    class MeUltrasonicSensor {
        public:
            MeUltrasonicSensor(uint8_t port);
			~MeUltrasonicSensor ();
            float distanceCm();
            float distanceInch();
			bool isWorking();
        private:
			uint32_t measure();
			mraa_gpio_context pin1;
			mraa_gpio_context pin2;
			uint8_t m_doWork;
			uint8_t m_InterruptCounter;
			long    m_RisingTimeStamp;
			long    m_FallingTimeStamp;
			/**
			 * ISR for the pulse signal
			 */
			static void signalISR(void *ctx);

    };
	class MeLineFollower {
        public:
            MeLineFollower(uint8_t port);
			~MeLineFollower ();
            uint8_t read();
			
        private:
			mraa_gpio_context pin1;
			mraa_gpio_context pin2;
    };
	class MeLightSensor {
        public:
            MeLightSensor(uint8_t port);
			~MeLightSensor();
            uint16_t read();
			
        private:
			mraa_aio_context pin;
    };
}
