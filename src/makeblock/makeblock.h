#pragma once
 
#include <stdio.h>
#include <math.h>
#include <mraa/aio.h>
#include <mraa/gpio.h>
#include <sys/time.h>
#include "servo.h"
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

/******************definitions for TM1637**********************/
#define   ADDR_AUTO 0x40   //Automatic address increment mode
#define   ADDR_FIXED 0x44   //Fixed address mode
#define   STARTADDR 0xc0  //start address of display register
#define   SEGDIS_ON 0x88   //diplay on
#define   SEGDIS_OFF 0x80   //diplay off
			/**** definitions for the clock point of the digit tube *******/
#define POINT_ON 1
#define POINT_OFF 0
			/**************definitions for brightness***********************/
#define BRIGHTNESS_0 0
#define BRIGHTNESS_1 1
#define BRIGHTNESS_2 2
#define BRIGHTNESS_3 3
#define BRIGHTNESS_4 4
#define BRIGHTNESS_5 5
#define BRIGHTNESS_6 6
#define BRIGHTNESS_7 7


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
            void run(int16_t pwm);
			
        private:
			mraa_pwm_context pin1;
			mraa_gpio_context  pin2;
    };
	
	class MeServoMotor {
        public:
            MeServoMotor(uint8_t port,uint8_t slot);
			~MeServoMotor ();
            void run(uint8_t angle);
		private:
			Servo *svr;
    };
	
	class MeStepperMotor {
        public:
            MeStepperMotor(uint8_t port);
			~MeStepperMotor ();
            void step();
			bool run();
			bool runSpeed();
			void setSpeed(uint16_t speed);
			void setMaxSpeed(float speed);
			void setAcceleration(float acceleration);
			void move(long distance);
			void moveTo(long distance);
			long distanceToGo();
			long currentPosition();
			unsigned long stepInterval();
			unsigned long ctime();
        private:
			mraa_gpio_context pin1;
			mraa_gpio_context pin2;
			struct timeval timer;
			static uint8_t _dir_data;
			static uint8_t _step_data;
			uint8_t _dir;          
			long _currentPos;     // Steps
			long _targetPos;      // Steps
			float _speed;         // Steps per second
			float _maxSpeed;
			float _acceleration;
			unsigned long _stepInterval;
			unsigned long _lastStepTime;
			/// The step counter for speed calculations
			long _n;
			/// Initial step size in microseconds
			float _c0;
			/// Last step size in microseconds
			float _cn;
			/// Min step size in microseconds based on maxSpeed
			float _cmin; // at max speed
			void computeNewSpeed();
    };
	class Me7SegmentDisplay{
		public:
			Me7SegmentDisplay(uint8_t port);
			~Me7SegmentDisplay ();
			void display(float value);
			void clear();
        private:
			void display(uint8_t DispData[]);
			void coding(uint8_t DispData[]);
			mraa_gpio_context pin1;
			mraa_gpio_context pin2;
			uint8_t Cmd_SetData;
			uint8_t Cmd_SetAddr;
			uint8_t Cmd_DispCtrl;
			void set(uint8_t brightness, uint8_t SetData, uint8_t SetAddr);
			void writeByte(uint8_t wr_data);
			void start(void);
			void stop(void);
			void write(uint8_t SegData[]);
			int16_t checkNum(float v,int16_t b);
	};
	class MeShutter{
		public:
			MeShutter(uint8_t port);
			~MeShutter ();
			void shotOnTime(long time);
			void shotOn();
			void shotOff();
			void focusOn();
			void focusOff();
        private:
			mraa_gpio_context pin1;
			mraa_gpio_context pin2;
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
			void setMode(uint8_t mode);
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
            uint16_t read(uint8_t mode=0);
			
        private:
			mraa_aio_context pin1;
			mraa_gpio_context pin2;
    };
	class MeGasSensor {
        public:
            MeGasSensor(uint8_t port);
			~MeGasSensor();
            uint16_t read(uint8_t mode=0);
			
        private:
			mraa_aio_context pin1;
			mraa_gpio_context pin2;
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
			static void signalISR(void *ctx);
			uint8_t m_doWork;
			uint8_t m_InterruptCounter;
			long    m_RisingTimeStamp;
			long    m_FallingTimeStamp;
			uint8_t humidity;
			uint8_t temperature;
	};
}
