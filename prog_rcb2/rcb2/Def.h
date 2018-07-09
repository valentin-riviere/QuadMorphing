/*
This file is part of QuadMorphing project
Copyright (C) 2018 - Institute of Movement Sciences (Marseille, FRANCE)

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef DEF_h
	#define DEF_h

	#include <math.h>

	/*************** MACRO FOR DEBUG ************/

	// #define ECHO_PRINTF   // Debug (not so useful)
	// #define ECHO_ONLY_GUM // Activate only echo for gumstix debug
	// #define ECHO_TIME_EXEC_RCB2	// Display execution time of differents rcb2 loop
	// #define ECHO_TIME_EXEC_PILOT  	// Display execution time of differents pilots loop (need disable ECHO_PRINTF)
	// #define ECHO_TIME_EXEC_AUTOPILOT // Display execution time for all functions in autopilot
	// #define RC_OFF  // If defined (manual mode = 0), no RC needed
	
	/********************************************/
	
	// Display data
	// #define ECHO_DISPLAY

	// Estimator (select only one)
	#define CF_ESTIMATOR
	// #define MAHONY_ESTIMATOR
	// Mahony filter
	#define KP_MAH 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
	#define KI_MAH 0.0f
	// Complementary filter
	#define rad2deg    57.295779513082323f
	#define IMU_2_rad  0.001065264436032f
	#define IMU_2_g    0.002395019531250f  
	#define Dt_CF      0.005f
	#define g_squared  96.2361f
	#define g_earth    9.81f
	#define g_double   19.62f
	#define deg2rad	   0.017453292519943f

	//  VBAT monitoring
	#define V_BATPIN A3
	#define V_PER_BIT			3.3/1023							// Volts/bit (measure [0 3.3V] on 10 bits)
	#define VBAT_SCALEFACTOR	4.85								// Vreal = VBAT_SCALEFACTOR x Vmeasure
	#define VBAT_LEVEL_MAX		16.8/(V_PER_BIT*VBAT_SCALEFACTOR)	// 16.8V MAX BAT
	#define VBAT_LEVEL_WA		14.4/(V_PER_BIT*VBAT_SCALEFACTOR)	// 14.0V MEDIUM BAT
	#define VBAT_LEVEL_NOK		13.0/(V_PER_BIT*VBAT_SCALEFACTOR)	// 13.0V EMPTY BAT
	#define FULL_BAT			0
	#define MEDIUM_BAT			1
	#define EMPTY_BAT			2

	// SAMPLE TIME (in us)
	#define STEP_SAMPLE_TIME 1250	// Period (in us) to execute controller
	#define PILOT_SAMPLE_TIME 5000
	#define RX_SAMPLE_TIME 20000
	#define BAT_SAMPLE_TIME 60000
	#define DISP_SAMPLE_TIME 1000000

	// roll pitch and yaw
	#define ROLL       0
	#define PITCH      1
	#define YAW        2

	// ******************
	// rc functions
	// ******************
	#define MINCHECK 1100
	#define MAXCHECK 1900

	/************** IMU **************/
	// #define DEBUG_IMU  // set to get Serial output for IMU debugging
	#define MPU6050_ADDRESS	0x68
	#define MPU9250_ADDRESS 0x69  // IMU Device address
	#define AK8963_ADDRESS 0x0C   // Address of magnetometer
	#define MS5611_ADDRESS 0x77   // Address of altimeter
	// #define MAG_USAGE	// Use Magnetometer (slow acquisition, doesn't work with CLOSED_LOOP)
	// #define MAG_CALIB // Calibration
	// #define ACC_CALIB
	#define ACC_BIAS_X 0.00f	// Bias for accelerometer (only used for manual and autopilot)
	#define ACC_BIAS_Y 0.05f
	#define ACC_BIAS_Z 0.03f
	#define USE_TEENSY_CAL	// Define to use Teensy calibration for gumstix data
	
	/************** I2C IMU **************/
	// See i2c_t3 library
	#define I2C_PINS I2C_PINS_18_19
	#define I2C_RATE I2C_RATE_400
	
	/************** UART1 GUMSTIX **************/
	#define UART1_SPEED 115200
	#define NB_DATA_TO_RECEIVE 4
	#define NB_MISSED_GUMSTIX 3		// Number of missing string before activating autopilot
	#define HEADER_RS232_ON			// Header = 170 (synchro stream for rs232)

	/************** PLATFORM **************/
	#define NB_ROTOR	4

	/************** AUTOPILOT **************/
	#define ROBOT_WEIGHT 0.733	// Robot weight for autopilot

	/**************** RX ****************/
	#define THROTTLEPIN                26
	#define ROLLPIN                    27
	#define PITCHPIN                   28
	#define YAWPIN                     29
	#define AUX1PIN                    30
	#define AUX2PIN                    31

	/*********** RC alias *****************/
	#define ROLL       0
	#define PITCH      1
	#define YAW        2
	#define THROTTLE   3
	#define AUX1       4
	#define AUX2       5
	
	/****************************    Motor maxthrottle    *******************************/
	/* this is the maximum value for the ESCs at full power, this value can be increased up to 2000 */
	#define MAXTHROTTLE	2000
	/****************************    Motor minthrottle    *******************************/
	/* Set the minimum throttle command sent to the ESC (Electronic Speed Controller)
	 This is the minimum value that allow motors to run at a idle speed  */
	#define MINTHROTTLE 1080
	//#define MINTHROTTLE	1080  // with the rotor speed controllers

	/****************************    Mincommand          *******************************/
	/* this is the value for the ESCs when they are not armed
		 in some cases, this value must be lowered down to 900 for some specific ESCs, otherwise they failed to initiate */
	#define MINCOMMAND	1000
	
	#define ARM_TIMEUP_SETPOINT 	1050	// [us] setpoint period which corresponds to a MIN_TIMEUP_PWM (rotors on in open loop with the ARM_ROTOR_PWM)

	#define MAX_TIMEUP_PWM	2000		// [us] Max PWM allowed to send to the ESC when regulation is on
	#define MIN_TIMEUP_PWM	1100		// [us] Min PWM allowed to send to the ESC when regulation is on
	#define ARM_ROTOR_PWM	1100		// [us] Mean PWM which turn on the rotation of a rotor
	#define ARM_ESC_PWM		1000		// [us] PWM needed to stop ESC beep...
	
	#define MAX_FREQ_SETPOINT	455 // [Hz] freq which corresponds to a MAX_TIMEUP_SETPOINT (158g)
	#define MIN_FREQ_SETPOINT	48	// [Hz] freq which corresponds to a MIN_TIMEUP_SETPOINT (20g)
	#define MAX_FREQ_SAT		470	// [Hz] 
	#define MIN_FREQ_SAT		35	// [Hz]
	
	/**************** Regulation ****************/	
	// #define DEBUG_REGUL
	#define US_2_FREQ_a	0.4522f	// a and b value with one magnet
	#define US_2_FREQ_b 449.0f	// y = a.x - b (x:us to y:Hz)

	#define IC_2_sec	0.000002	// Convert IC measure (half-period in us) in sec

	#define ROTOR_1 0
	#define ROTOR_2 1
	#define ROTOR_3 2
	#define ROTOR_4 3

	#define NOT_ARMED		0
	#define ARMED			1
	#define OPEN_LOOP		0
	#define CLOSED_LOOP		1
	#define IDLE			2
	
	#define ROTOR_INIT_TIME_us 1000000 // Time needed to make the rotor start
	#define ESC_INIT_TIME_us 	500000	// Time needed to make the rotor start
	
	// #define TACHY_ON 			// If tachys are presented
	#define REGUL_STATUS OPEN_LOOP	// OPEN_LOOP or CLOSED_LOOP (need tachys on) or IDLE // WARNING in closed loop: disable mag acquisition
	
	#define PWM_SP				{1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000}	// Correspond to Omega SP below
	#define OMEGA_SP_HZ			{48, 115, 169, 212, 251, 287, 330, 379, 425, 455}				// Omega SetPoints (in Hz) and A/b for LookUp Table
	#define A_HZ_TO_US			{1.505, 1.843, 2.322, 2.593, 2.787, 2.301, 2.029, 2.204, 3.277}
	#define b_HZ_TO_US			{1027, 989, 908, 850, 802, 941, 1030, 964, 509}
	
	#define NB_MAGNET_PER_ROTOR 6
	
	/**************** Stabilization during manual control ****************/
	// #define DEBUG_STAB_ATT	// Debug stabilization attitude
	#define DEBUG_STAB_TIME 300 // Time between two prints
	
	/**************** Filter Regulation ****************/
	#define FILTER_ORDER 		2			// 0: no filtering, 1: first order filter (fc = 1/30kHz), 2: second order filter (fc = 1/30kHz, m=1)
	#define LPF_FREQUENCY 		7.0f		// [Hz] cut-off frequency of the setpoint's low pass filter
	#define LPF_FREQUENCY_MEAS 	100.0f		// [Hz] cut-off frequency of the setpoint's low pass filter
	#define SIZE_LOOKUP_FF		10
	
	#define SAMPLE_TIME_s	0.00125

	/**************** Tachys ****************/
	// #define DEBUG_TACHYS
	#define PIN_T1 20	// Front
	#define PIN_T2 21	// Left
	#define PIN_T3 22	// Rear
	#define PIN_T4 23	// Right

	/**************** MOTORS ****************/
	#define PIN_M1 6	// NE
	#define PIN_M2 5	// NW
	#define PIN_M3 4	// SW
	#define PIN_M4 3	// SE

	#define BITS_PWM_RESOLUTION 11	// Resolution PWM for motor command
	#define PWM_CONS_MAX 2047 		// = 2^BITS_PWM_RESOLUTION-1

	#define OC_UPDATE 2000	//Refresh rate (in us) for Ouput Command

	/**************** LED ****************/
	#define PIN_LED 13

#endif
