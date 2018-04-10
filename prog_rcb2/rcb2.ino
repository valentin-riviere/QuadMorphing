/*
This file is part of QuadMorphing project
Copyright (C) 2018 - Institute of Movement Sciences (Marseille, FRANCE)

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/* memory:

float                 : 4 bytes
long                  : 4 bytes
int,   unsigned int   : 2 bytes
short, unsigned short : 2 bytes 
char,  unsigned char  : 1 byte

int32_t, uint32_t : 4 bytes
int16_t, uint16_t : 2 bytes
int8_t, uint8_t   : 1 bytes

*/

#include "config.h"
#include "Def.h"
#include "Regulation.h"
#include "Tachys.h"

// **************
// Global Variables
// **************
static uint8_t init_OK = 0;
static uint8_t ManualMode = 1;
static uint8_t MotorsArmed = 0;
static int8_t n_offset = 0;
static int16_t DataToSend[23] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint32_t currentTime = 0;
volatile struct RegulSignals RegulSig;
volatile struct PID Pid[NB_ROTOR];
volatile struct FilterSignals FilterSig[NB_ROTOR];
volatile struct FilterSignals FilterMeas[NB_ROTOR];
#ifdef DEBUG_REGUL
	volatile uint16_T Debug1[NB_ROTOR] = {0, 0, 0, 0};
	volatile uint16_T Debug2[NB_ROTOR] = {0, 0, 0, 0};
	volatile uint16_T Debug3[NB_ROTOR] = {0, 0, 0, 0};
#endif
volatile uint32_t TachyRisingPeriod[NB_ROTOR] = {0,0,0,0};	// Time between two fronts of tachy
volatile int16_t SetPoint_us[NB_ROTOR]; 					// SetPoints (inputs for regulation) (in us)
#ifdef ECHO_DISPLAY
	float dispRotorSpeed[NB_ROTOR] = {0.0, 0.0, 0.0, 0.0};
#endif

// **************
// Complementary Filter
// **************
static float Omega_hat[3]   = {0.0, 0.0, 0.0};  // [rad/s] debiased 
static float B_hat[3]       = {0.0, 0.0, 0.0};   // [rad/s] gyro's bias
static float Q_hat[4]       = {1.0, 0.0, 0.0, 0.0};
static float EulerAngles[3] = {0.0, 0.0, 0.0};
static float m_bar[3]       = {1.0, 0.0, 0.0};  // fictious magnitude reference

static float k_a[3] = {1.0, 1.0, 0.0};
static float k_b[3] = {0.1, 0.1, 0.1};
static float k_m[3] = {0.0, 0.0, 5.0};

// **************
// Attitude stabilization
// **************
static float EulerSetPoint[3] = {0.0, 0.0, 0.0};
static float ThrottleSetPoint = 0.0;

// **************
// gyro+acc IMU
// **************
uint8_t rawADC[14];			// Raw data from IMU (6 bytes for acc, 2 bytes for temp, 6 bytes for gyro)
float accADC[3];
float gyroADC[3];
float magADC[3];

// ************
// RC functions
// ************
volatile int16_t failsafeCnt = 0;
static int16_t rcData[8];          // interval [1000;2000]

// *************************
// motor and servo functions
// *************************
static int16_t motor[NB_ROTOR];

// *************************
// Battery functions
// *************************
static uint8_t BatteryLevel = FULL_BAT;
static int16_t vbatRaw = 0;

// *************************
// IC tachys
// *************************
volatile uint16_t TachyNbOverFlow[NB_ROTOR] = {0,0,0,0};

// *************************
// Timers
// *************************
IntervalTimer timer_refresh_OC;		// Refresh output
IntervalTimer timer_overflow_IC;	// Overflow IC

// *************************
// To Display Time Execution
// *************************
#ifdef ECHO_TIME_EXEC_PILOT
	uint32_t t_exec_rx_pilot_max = 0;
	uint32_t t_exec_gum_max = 0;
	uint32_t t_exec_ap_max = 0;
#endif
#ifdef ECHO_TIME_EXEC_RCB2
	int32_t t_exec_oneStep = 0;
	int32_t t_exec_Pilot_loop = 0;
	int32_t t_exec_RX_loop = 0;
	int32_t t_exec_CheckBattery = 0;
	int32_t t_exec_DisplayMenu = 0;
	int32_t t_exec_max_oneStep = 0;
	int32_t t_exec_max_Pilot_loop = 0;
	int32_t t_exec_max_RX_loop = 0;
	int32_t t_exec_max_CheckBattery = 0;
	int32_t t_exec_max_DisplayMenu = 0;
#endif

// *************************
// For polling
// *************************
uint32_t PilotTime  = 0;
uint32_t RxTime     = 0;
uint32_t VbatTime   = 0;
uint32_t StepTime   = 0;
uint32_t DispTime   = 0;

/**************************************************************************************/
/**************  PRINCIPAL INTERRUPT BASED ON TIMER 1: EACH Ts [ms]  ******************/
/**************************************************************************************/

void setup()
{
	led_init();
	Serial.begin(921600);        
	Serial1.begin(UART1_SPEED);

	led_blink_1s();
	Serial.println("Start...");

	initSensors();
	configureReceiver();
	initMotors();
	initRegul();
	setupTachys();

	Serial.println("Init...OK");

	led_turn_on();	// If it's ON, normal execution

	currentTime = micros();
}

void loop()
{
	// each 1.25ms, 800Hz, Regulation Loop
	if (currentTime >= StepTime)
	{
		StepTime = currentTime + STEP_SAMPLE_TIME;

		one_step();

		#ifdef ECHO_TIME_EXEC_RCB2
			t_exec_oneStep = micros()-currentTime;
			if (t_exec_max_oneStep < t_exec_oneStep)
				t_exec_max_oneStep = t_exec_oneStep;
		#endif
	}
	// each 5ms, 200Hz, Pilot Loop
	else if (currentTime >= PilotTime)
	{
		PilotTime = currentTime + PILOT_SAMPLE_TIME;

		Pilot_loop();

		#ifdef ECHO_TIME_EXEC_RCB2
			t_exec_Pilot_loop = micros()-currentTime;
			if (t_exec_max_Pilot_loop < t_exec_Pilot_loop)
				t_exec_max_Pilot_loop = t_exec_Pilot_loop;
		#endif
	}

	// each 20ms, 50Hz, RX Loop
	else if (currentTime >= RxTime)
	{
		RxTime = currentTime + RX_SAMPLE_TIME;

		#ifndef RC_OFF
			RX_loop();

			#ifdef ECHO_TIME_EXEC_RCB2
				t_exec_RX_loop = micros()-currentTime;
				if (t_exec_max_RX_loop < t_exec_RX_loop)
					t_exec_max_RX_loop = t_exec_RX_loop;
			#endif
		#else
			ManualMode = 0;
		#endif
	}

	// each 60ms, ~=17Hz, Battery Check Loop
	else if (currentTime >= VbatTime)
	{
		VbatTime = currentTime + BAT_SAMPLE_TIME;

		CheckBattery();

		#ifdef ECHO_TIME_EXEC_RCB2
			t_exec_CheckBattery = micros()-currentTime;
			if (t_exec_max_CheckBattery < t_exec_CheckBattery)
				t_exec_max_CheckBattery = t_exec_CheckBattery;
		#endif
	}
	
	#ifdef ECHO_DISPLAY
		// each 1s, 1Hz, Display Loop
		if (currentTime >= DispTime)
		{
			DispTime = currentTime + DISP_SAMPLE_TIME;

			DisplayMenu();

			#ifdef ECHO_TIME_EXEC_RCB2
				t_exec_DisplayMenu = micros()-currentTime;
				if (t_exec_max_DisplayMenu < t_exec_DisplayMenu)
					t_exec_max_DisplayMenu = t_exec_DisplayMenu;

				DisplayTimeExecRCB2();
			#endif
		}
	#endif

	currentTime = micros();
}

void CheckBattery(void)
{
	static uint8_t ind = 0;
	static int16_t vbatRawArray[8] = {VBAT_LEVEL_MAX, VBAT_LEVEL_MAX, VBAT_LEVEL_MAX, VBAT_LEVEL_MAX, VBAT_LEVEL_MAX, VBAT_LEVEL_MAX, VBAT_LEVEL_MAX, VBAT_LEVEL_MAX};

	vbatRawArray[(ind++)%8] = (int16_t)(analogRead(V_BATPIN));

	vbatRaw = 0;
	for (uint8_t i=0;i<8;i++)
	{
		vbatRaw += vbatRawArray[i];
	}
	vbatRaw = vbatRaw / 8;

	if(vbatRaw <= VBAT_LEVEL_NOK)
	{
		BatteryLevel = EMPTY_BAT;
		led_sw_state();
	}
	else if(vbatRaw <= VBAT_LEVEL_WA)
	{
		BatteryLevel = EMPTY_BAT;
		led_turn_on();
	}
	else
	{
		BatteryLevel = FULL_BAT;
		led_turn_off();
	}
}

void one_step()
{
	uint8_t i;

	// Convert set point
	for (i=0; i<NB_ROTOR; i++)
	{
		// get setpoint
		RegulSig.RotorSpeed_SetPoint[i] = us2freq_table((float)SetPoint_us[i]); // setpoint value in Hz
		// get measured frequency
		RegulSig.Previous2_Measured_RotorSpeed[i] = RegulSig.Previous_Measured_RotorSpeed[i];
		RegulSig.Previous_Measured_RotorSpeed[i] = RegulSig.Measured_RotorSpeed[i];
		RegulSig.Measured_RotorSpeed[i] = (1.0/(float)(IC_2_sec*NB_MAGNET_PER_ROTOR*((float)TachyRisingPeriod[i])));

		#ifdef ECHO_DISPLAY
			dispRotorSpeed[i] = RegulSig.Measured_RotorSpeed[i];
		#endif
	}

	if (RegulSig.RegulState != IDLE)	// If regulation is ON
	{
		for (i=0; i<NB_ROTOR; i++)
		{
			if (SetPoint_us[i] >= MINTHROTTLE)
			{
				if (RegulSig.RotorStatus[i] == ARMED) // if rotor is ON...
				{
					if(RegulSig.RegulState == CLOSED_LOOP)
					{
						// Determine RegulSig.TimeUp_PWM_ESC
						RegulRotorSpeed(i);
					}
					else if (RegulSig.RegulState == OPEN_LOOP)
					{
						RegulSig.TimeUp_PWM_ESC[i] = SetPoint_us[i];
					}
				}
				else
				{
					// initiate the rotor
					StartRotor(i);
				}
			}
			else if (SetPoint_us[i] >= ARM_TIMEUP_SETPOINT)
			{
				StartRotor(i);
			}
			else if (SetPoint_us[i] >= MINCOMMAND)
			{
				StopRotor(i);
			}
			else
			{
// char str[100];
// sprintf((char*)str, "Rotor %d: Invalid SetPoint (%f us)", i+1, (double)SetPoint_us[i]);
// Serial.println(str);
			}
		}
	}
}