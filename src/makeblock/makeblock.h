/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * \class MakeBlock for electronics modular
 * \brief   Driver for electronics modular device.
 * @file    makeblock.h
 * @author  MakeBlock
 * @version V1.0.0
 * @date    2016/02/23
 * @brief   Header for for makeblock.cpp module
 *
 * \par Copyright
 * This software is Copyright (C), 2012-2016, MakeBlock. Use is subject to license \n
 * conditions. The main licensing options available are GPL V2 or Commercial: \n
 *
 * \par Open Source Licensing GPL V2
 * This is the appropriate option if you want to share the source code of your \n
 * application with everyone you distribute it to, and you also want to give them \n
 * the right to share who uses it. If you wish to use this software under Open \n
 * Source Licensing, you must contribute all your source code to the open source \n
 * community in accordance with the GPL Version 2 when your application is \n
 * distributed. See http://www.gnu.org/copyleft/gpl.html
 *
 * \par Description
 * This file is a drive for makeblock electronics modular device, It supports sensor device
 * V2.2 provided by the MakeBlock. The electronics modular used RJ25 Interface connect sensor device
 * on Me UNO shield expansion board
 *
 * \par Method List:
 *
 *    1. void MeLineFollower::setpin(uint8_t Sensor1,uint8_t Sensor2)
 *    2. uint8_t MeLineFollower::readSensors(void)
 *    3. bool MeLineFollower::readSensor1(void)
 *    4. bool MeLineFollower::readSensor1(void)
 *
 * \par History:
 * <pre>
 * `<Author>`         `<Time>`        `<Version>`        `<Descr>`
 *  adream            2016/02/23       1.0.0            build the new lib.
 * </pre>
 *
 */
#pragma once

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <mraa/aio.h>
#include <mraa/gpio.h>
#include <mraa/uart.h>
#include <mraa/i2c.h>
#include <sys/time.h>
#include "servo.h"
#include "mpu60x0.h"
/********************definitions pin level*/
#define HIGH                   1
#define LOW                    0
/********************definitions RJ25 port********************/
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

/**************definitions for MeHumiture  return ***********************/
#define DHTLIB_OK				0
#define DHTLIB_ERROR_CHECKSUM	-1
#define DHTLIB_ERROR_TIMEOUT	-2

/// @brief Class for RGB Led Module
struct cRGB
{
  uint8_t g;
  uint8_t r;
  uint8_t b;
};

namespace upm {
	
/**
 * Class: MePort
 *
 * \par Description
 * Declaration of Class MePort
 */
	class MePort{
		public:
/**
 * Alternate Constructor which can call your own function to map the MePort to arduino port,
 * no pins are used or initialized here
 */
			MePort();

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
			uint8_t getPin(uint8_t port,uint8_t slot);
			
		private:
/**
 *  \par Description
 *  Variables used to store the port
 */
			uint8_t *ports;
	};
	
/**
 * Class: MeDCMotor
 * \par Description
 * Declaration of Class MeDCMotor.
 */
	class MeDCMotor {
        public:
/**
 * Alternate Constructor which can call your own function to map the DC motor to arduino port
 * pin will be used here
 * \param[in]
 *   port - RJ25 port from PORT_9 to PORT_10
 */
            MeDCMotor(uint8_t port);
			
/**
 * Uninstall Constructor when you call function  ~MeDCMotor() 
 * \param[in]
 *   None
 */		
 
			~MeDCMotor ();
			
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
            void run(float pwm);	
			
        private:
			mraa_pwm_context pin1;  //used pwm pin to control dcmotor speed
			mraa_gpio_context  pin2;//used gpio output pin to control dcmotor direction
    };
	
/**
 * Class: MeServoMotor
 *
 * \par Description
 * Declaration of Class MeServoMotor
 */
	class MeServoMotor {
        public:
/**
 * Alternate Constructor which can call your own function to map the Servo Motor to arduino port
 * \param[in]
 *   port - RJ25 port only PORT_4 and solt is 1
 */		
            MeServoMotor(uint8_t port,uint8_t slot);
			
/**
 * Uninstall Constructor when you call function  ~MeServoMotor() 
 * \param[in]
 *   None
 */		
			~MeServoMotor ();
			
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
            void run(uint8_t angle);
			
		private:
			Servo *svr;//The definition of a pointer object actuator
    };
	
/**
 * Class: MeStepperMotor
 * \par Description
 * Declaration of Class MeStepperMotor.
 */
	class MeStepperMotor {
        public:
/**
 * Alternate Constructor which can call your own function to map the stepper to arduino port,
 * pins are used or initialized here.
 * \param[in]
 *   port
 */
            MeStepperMotor(uint8_t port);
			
/**
 * Uninstall Constructor when you call function  ~MeStepperMotor() 
 * \param[in]
 *   None
 */	
			~MeStepperMotor ();
			
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
            void step();
			
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
			bool run();
			
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
			bool runSpeed();
			
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
			void setSpeed(uint16_t speed);
			
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
			void setMaxSpeed(float speed);
			
/**
 * \par Function
 *    setAcceleration
 * \par Description
 *    Set Acceleration for Stepper.
 * \param[in]
 *    acceleration - The acceleration for Stepper.
 * \par Output
 *    None
 * \par Return
 *    None
 * \par Others
 *    None
 */
			void setAcceleration(float acceleration);
			
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
			void move(long distance);
			
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
			void moveTo(long distance);
			
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
			long distanceToGo();
			
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
			long currentPosition();
			
			
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
			unsigned long stepInterval();
			
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
			float _maxSpeed;      //set max speed
			float _acceleration;  //set acceleration
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
			void computeNewSpeed();
    };
	
/**
 * Class: Me7SegmentDisplay
 * \par Description
 * Declaration of Class Me7SegmentDisplay.
 */
	class Me7SegmentDisplay{
		public:
/**
 * Alternate Constructor which can call your own function to map the 7-Segment display to arduino port,
 * the slot1 will be used for data pin and slot2 will used for clk pin.
 * \param[in]
 *   port - RJ25 blue port
 */	
			Me7SegmentDisplay(uint8_t port);
			
/**
 * Uninstall Constructor when you call function  ~Me7SegmentDisplay() 
 * \param[in]
 *   None
 */			
			~Me7SegmentDisplay ();
			
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
			void display(float value);
			
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
			void clear();
			
        private:
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
			void display(uint8_t DispData[]);
			
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
			void coding(uint8_t DispData[]);
			
			mraa_gpio_context pin1;
			mraa_gpio_context pin2;
			uint8_t Cmd_SetData;
			uint8_t Cmd_SetAddr;
			uint8_t Cmd_DispCtrl;

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
			void set(uint8_t brightness, uint8_t SetData, uint8_t SetAddr);
			
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
			void writeByte(uint8_t wr_data);
			
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
			void start(void);
			
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
			void stop(void);	
			
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
			void write(uint8_t SegData[]);
			
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
			int16_t checkNum(float v,int16_t b);
	};
	
/**
 * Class: MeShutter
 * \par Description
 * Declaration of Class MeShutter.
 */
	class MeShutter{
		public:
/**
 * Alternate Constructor which can call your own function to map the MeShutter to arduino port,
 * and the shot and focus PIN will be set LOW
 * \param[in]
 *   port - RJ25 blue port(3,7,8) 
 */		
			MeShutter(uint8_t port);
/**
 * Uninstall Constructor when you call function  ~MeShutter() 
 * \param[in]
 *   None
 */			
			~MeShutter();
			
			void shotOnTime(long time);
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
			void shotOn();
			
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
			void shotOff();
			
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
			void focusOn();
			
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
			void focusOff();
			
        private:
			mraa_gpio_context pin1;
			mraa_gpio_context pin2;
	};
	
/**
 * Class: MeUltrasonicSensor
 * \par Description
 * Declaration of Class MeUltrasonicSensor.
 */	
    class MeUltrasonicSensor {
        public:
/**
 * Alternate Constructor which can call your own function to map the ultrasonic Sensor to arduino port
 * \param[in]
 *   port - RJ25 yellow port 
 */		
            MeUltrasonicSensor(uint8_t port);
/**
 * Uninstall Constructor when you call function  ~MeUltrasonicSensor() 
 * \param[in]
 *   None
 */			
			~MeUltrasonicSensor ();
			
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
            float distanceCm();

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
            float distanceInch();
			
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
			bool isWorking();
			
        private:
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
	
/*	
 * Class: MeLineFollower
 * \par Description
 * Declaration of Class MeLineFollower.
 */	
	class MeLineFollower {
        public:
/**
 * Alternate Constructor which can call your own function to map the line follwer device to arduino port,
 * no pins are used or initialized here.
 * \param[in]
 *   None
 */		
            MeLineFollower(uint8_t port);
			
/**
 * Uninstall Constructor when you call function  ~MeLineFollower() 
 * \param[in]
 *   None
 */				
			~MeLineFollower ();
			
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
            uint8_t read();
			
        private:
			mraa_gpio_context pin1;
			mraa_gpio_context pin2;
    };
	
/**
 * Class: MeLimitSwitch
 * \par Description
 * Declaration of Class MeLimitSwitch.
 */	
	class MeLimitSwitch {
        public:
/**
 * Alternate Constructor which can call your own function to map the limit switch to arduino port,
 * no pins are used or initialized here.
 * \param[in]
 *   None
 */		
            MeLimitSwitch(uint8_t port);
			
/**
 * Uninstall Constructor when you call function  ~MeLimitSwitch() 
 * \param[in]
 *   None
 */			
			~MeLimitSwitch ();
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
            uint8_t read(uint8_t slot);
			
        private:
			mraa_gpio_context pin1;
			mraa_gpio_context pin2;
    };
	
/**
 * Class: MeTouchSensor
 * \par Description
 * Declaration of Class MeTouchSensor.
 */	
	class MeTouchSensor {
        public:
/**
 * Alternate Constructor which can call your own function to map the touch Sensor to arduino port
 * \param[in]
 *   port - RJ25 blue port
 */		
            MeTouchSensor(uint8_t port);

/**
 * Uninstall Constructor when you call function  ~MeLimitSwitch() 
 * \param[in]
 *   None
 */				
			~MeTouchSensor ();

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
            uint8_t read();
			
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
			void setMode(uint8_t mode);
			
        private:
			mraa_gpio_context pin1;
			mraa_gpio_context pin2;
    };
	
/**
 * Class: MePIRMotionSensor
 * \par Description
 * Declaration of Class MePIRMotionSensor.
 */	
	class MePIRMotionSensor {
        public:
/**
 * Alternate Constructor which can call your own function to map the Motion Sensor device to arduino port
 * \param[in]
 *   port - RJ25 blue port 
 */		
            MePIRMotionSensor(uint8_t port);

/**
 * Uninstall Constructor when you call function  ~MePIRMotionSensor() 
 * \param[in]
 *   None
 */			
			~MePIRMotionSensor ();

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
            uint8_t read();
			
        private:
			mraa_gpio_context pin;
    };
	
/**
 * Class: MeLightSensor
 * \par Description
 * Declaration of Class MeLightSensor.
 */	
	class MeLightSensor {
        public:
/**
 * Alternate Constructor which can call your own function to map the light sensor to arduino port
 * \param[in]
 *   port - RJ25 black port
 */		
            MeLightSensor(uint8_t port);

/**
 * Uninstall Constructor when you call function  ~MeLightSensor() 
 * \param[in]
 *   None
 */			
			~MeLightSensor();
			
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
            uint16_t read();
			
        private:
			mraa_aio_context pin;
    };

/**
 * Class: Me4Button
 * \par Description
 * Declaration of Class Me4Button
 */	
	class Me4Button {
        public:
		
/**
 *  Alternate Constructor which can call your own function to map the Me4Button to arduino port, \n
 * \param[in]
 *    port - RJ25 black port
 */		
            Me4Button(uint8_t port);
/**
 * Uninstall Constructor when you call function  ~Me4Button() 
 * \param[in]
 *   None
 */				
			~Me4Button();

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
            uint8_t read();
			
        private:
			mraa_aio_context pin;
    };
	
/**
 * Class: MeSoundSensor
 * \par Description
 * Declaration of Class MeSoundSensor.
 */	
	class MeSoundSensor {
        public:
/**
 * Alternate Constructor which can call your own function to map the Sound Sensor to arduino port
 * \param[in]
 *   port - RJ25 black port
 */		
            MeSoundSensor(uint8_t port);
			
/**
 * Uninstall Constructor when you call function  ~MeSoundSensor() 
 * \param[in]
 *   None
 */			
			~MeSoundSensor();
			
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
            uint16_t read();
			
        private:
			mraa_aio_context pin;
    };
	
/**
 * Class: MePotentiometer
 * \par Description
 * Declaration of Class MePotentiometer.
 */	
	class MePotentiometer {
        public:
/**
 * Alternate Constructor which can call your own function to map the Me potentiometer device to arduino port
 * \param[in]
 *   port - RJ25 black port
 */		
            MePotentiometer(uint8_t port);
			
/**
 * Uninstall Constructor when you call function  ~MePotentiometer() 
 * \param[in]
 *   None
 */			
			~MePotentiometer();
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
            uint16_t read();
			
        private:
			mraa_aio_context pin;
    };
	
/**
 * Class: MeJoystick
 * \par Description
 * Declaration of Class MeJoystick.
 */	
	class MeJoystick {
        public:
/**
 * Alternate Constructor which can call your own function to map the MeJoystick to arduino port.
 * \param[in]
 *   port - RJ25 black port
 */		
            MeJoystick(uint8_t port);
			
/**
 * Uninstall Constructor when you call function  ~MeJoystick() 
 * \param[in]
 *   None
 */				
			~MeJoystick();
			
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
            uint16_t read(uint8_t axis);
			
        private:
			mraa_aio_context pin1;
			mraa_aio_context pin2;
    };
	
/**
 * Class: MeFlameSensor
 * \par Description
 * Declaration of Class MeFlameSensor
 */	
	class MeFlameSensor {
        public:
/**
 * Alternate Constructor which can call your own function to map the flame snesor to arduino port
 * \param[in]
 *   port - RJ25 black port
 */		
            MeFlameSensor(uint8_t port);

/**
 * Uninstall Constructor when you call function  ~MeFlameSensor() 
 * \param[in]
 *   None
 */			
			~MeFlameSensor();
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
            uint16_t read(uint8_t mode=0);
			
        private:
			mraa_aio_context pin1;
			mraa_gpio_context pin2;
    };
	
/**
 * Class: MeGasSensor
 * \par Description
 * Declaration of Class MeGasSensor
 */	
	class MeGasSensor {
        public:
/**
 * Alternate Constructor which can call your own function to map the gas snesor to arduino port
 * \param[in]
 *   port - RJ25 black port
 */		
            MeGasSensor(uint8_t port);

/**
 * Uninstall Constructor when you call function  ~MeGasSensor() 
 * \param[in]
 *   None
 */			
			~MeGasSensor();
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
            uint16_t read(uint8_t mode=0);
			
        private:
			mraa_aio_context pin1;
			mraa_gpio_context pin2;
    };
	
/**
 * Class: MeHumiture
 * \par Description
 * Declaration of Class MeHumiture.
 */	
	class MeHumiture {
		public:
/**
 * Alternate Constructor which can call your own function to map the humiture sensor to arduino port,
 * the slot2 pin will be used here since specify slot is not be set.
 * \param[in]
 *   port - RJ25 yellow port
 */		
			MeHumiture(uint8_t port);
			
/**
 * Uninstall Constructor when you call function  ~MeHumiture() 
 * \param[in]
 *   None
 */			
			~MeHumiture();

/**
 * \par Function
 *   update
 * \par Description
 *   Use this function to update the sensor data
 * \par Output
 *   None
 * \return
 *   read state
 * \par Others
 *   None
 */			
			int update();
			
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
			uint8_t getHumidity();
			
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
			uint8_t getTemperature();

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
			double getKelvin();
			
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
			double getFahrenheit();
			
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
			double getDewPoint();

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

/**
 * Class: MeGyro
 * \par Description
 * Declaration of Class MeGyro
 */	
	class MeGyro {
        public:
/**
 * Alternate Constructor which can call your own function to map the MeGyro to arduino port,
 * no pins are used or initialized here
 */		
            MeGyro();
			
/**
 * Uninstall Constructor when you call function  ~MeGyro() 
 * \param[in]
 *   None
 */			
			~MeGyro();
			
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
			void begin();
			
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
            void update();
			
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
            double read(uint8_t axis);
			
        private:
			MPU60X0 mpu6050;
			float gx,gy,gz;
    };

/**
 * Class: MeTftLCD
 * \par Description
 * Declaration of Class MeTftLCD
 */		
	class MeTftLCD {
        public:
/**
 * Alternate Constructor which can call your own function to map the TftLCD to arduino port,
 * no pins are used or initialized here
 */			
            MeTftLCD();
/**
 * Uninstall Constructor when you call function  ~MeTftLCD() 
 * \param[in]
 *   None
 */			
			~MeTftLCD();
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
            void send(char *str);
			
        private:
			mraa_uart_context uart;
};

/**
 * Class: MeRGBLed
 *
 * \par Description
 * Declaration of Class MeRGBLed
 */
	class MeRGBLed {
		public:
/**
 * Alternate Constructor which can call your own function to map the MeRGBLed to arduino port,
 * it will assigned the LED display buffer and initialization the GPIO of LED lights. The slot2
 * will be used here, and the default number of light strips is 32.
 * \param[in]
 *   port - RJ25 yellow port 
 */
			MeRGBLed(uint8_t port);
			
/**
 * Uninstall Constructor when you call function  ~MeHumiture() 
 * \param[in]
 *   None
 */			
			~MeRGBLed();
			
/**
 * \par Function
 *   setNumber
 * \par Description
 *   Assigned the LED display buffer by the LED number
 * \param[in]
 *   num_leds - The LED number you used
 * \par Output
 *   None
 * \return
 *   None
 * \par Others
 *   None
 */
			 void setNumber(uint8_t num_led);
			 
/**
 * \par Function
 *   setColorAt
 * \par Description
 *   Set the LED color for any LED.
 * \param[in]
 *   index - The LED index number you want to set its color
 * \param[in]
 *   red - Red values
 * \param[in]
 *   green - green values
 * \param[in]
 *   blue - blue values
 * \par Output
 *   None
 * \return
 *   TRUE: Successful implementation
 *   FALSE: Wrong execution
 * \par Others
 *   The index value from 0 to the max.
 */
bool setColorAt(uint8_t index, uint8_t red, uint8_t green, uint8_t blue);

		private:
			mraa_gpio_context pin1;
			uint16_t count_led;
			uint8_t *pixels;
			const volatile uint8_t *ws2812_port;
			volatile uint8_t *ws2812_port_reg;
			uint8_t pinMask;
	
    };
}
