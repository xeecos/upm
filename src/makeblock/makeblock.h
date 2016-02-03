#pragma once
 
#include <stdio.h>
#include <math.h>
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
	
	class MeDCMotor {
        public:
            MeDCMotor(uint8_t port);
			~MeDCMotor ();
            void run(uint8_t pwm);
			
        private:
			mraa_gpio_context pin1;
			mraa_gpio_context pin2;
    };
	
	class MeServoMotor {
        public:
            MeServoMotor(uint8_t port,uint8_t slot);
			~MeServoMotor ();
            void run(uint8_t angle);
			
        private:
			mraa_gpio_context pin;
    };
	
	class MeStepperMotor {
        public:
            MeStepperMotor(uint8_t port);
			~MeStepperMotor ();
            void step(uint8_t dir);
			void run();
			void setSpeed(uint16_t speed);
			void setAcc(uint16_t speed);
			void move(long distance);
			void moveTo(long distance);
			
        private:
			mraa_gpio_context pin1;
			mraa_gpio_context pin2;
    };
	class Me7SegmentDisplay{
		public:
			Me7SegmentDisplay(uint8_t port);
			~Me7SegmentDisplay ();
			void display(float value);
        private:
			mraa_gpio_context pin1;
			mraa_gpio_context pin2;
	}
	class Me7SegmentDisplay{
		public:
			Me7SegmentDisplay(uint8_t port);
			~Me7SegmentDisplay ();
			void display(float value);
        private:
			mraa_gpio_context pin1;
			mraa_gpio_context pin2;
	}
	class MeShutter{
		public:
			MeShutter(uint8_t port);
			~MeShutter ();
			void shotOn();
			void shotOff();
			void focusOn();
			void focusOff();
        private:
			mraa_gpio_context pin1;
			mraa_gpio_context pin2;
	}
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
	class MeLimitSwitch {
        public:
            MeLimitSwitch(uint8_t port);
			~MeLimitSwitch ();
            uint8_t read(uint8_t slot);
			
        private:
			mraa_gpio_context pin1;
			mraa_gpio_context pin2;
    };
	class MeTouchSensor {
        public:
            MeTouchSensor(uint8_t port);
			~MeTouchSensor ();
            uint8_t read();
			
        private:
			mraa_gpio_context pin1;
			mraa_gpio_context pin2;
    };
	class MePIRMotionSensor {
        public:
            MePIRMotionSensor(uint8_t port);
			~MePIRMotionSensor ();
            uint8_t read();
			
        private:
			mraa_gpio_context pin;
    };
	class MeLightSensor {
        public:
            MeLightSensor(uint8_t port);
			~MeLightSensor();
            uint16_t read();
			
        private:
			mraa_aio_context pin;
    };
	class MeSoundSensor {
        public:
            MeSoundSensor(uint8_t port);
			~MeSoundSensor();
            uint16_t read();
			
        private:
			mraa_aio_context pin;
    };
	class MePotentiometer {
        public:
            MePotentiometer(uint8_t port);
			~MePotentiometer();
            uint16_t read();
			
        private:
			mraa_aio_context pin;
    };
	class MeJoystick {
        public:
            MeJoystick(uint8_t port);
			~MeJoystick();
            uint16_t read(uint8_t axis);
			
        private:
			mraa_aio_context pin1;
			mraa_aio_context pin2;
    };
	class MeFlameSensor {
        public:
            MeFlameSensor(uint8_t port);
			~MeFlameSensor();
            uint16_t read(uint8_t threshold);
			
        private:
			mraa_aio_context pin1;
			mraa_aio_context pin2;
    };
	class MeGasSensor {
        public:
            MeGasSensor(uint8_t port);
			~MeGasSensor();
            uint16_t read(uint8_t threshold);
			
        private:
			mraa_aio_context pin1;
			mraa_aio_context pin2;
    };
	class MeHumiture {
		public:
			MeHumiture(uint8_t port);
			~MeHumiture();
			void update();
			uint8_t getHumidity();
			uint8_t getTemperature();
			double getKelvin();
			double getFahrenheit();
			double getDewPoint();
			double getPointFast();
		private:
			mraa_gpio_context pin1;
			mraa_gpio_context pin2;
			uint8_t humidity;
			uint8_t temperature;
	};
}
