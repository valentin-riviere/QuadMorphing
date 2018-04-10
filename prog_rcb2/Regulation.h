/*
This file is part of QuadMorphing project
Copyright (C) 2018 - Institute of Movement Sciences (Marseille, FRANCE)

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef __REGULATION_H__
#define __REGULATION_H__

#include <Arduino.h>

/*********************************** GLOBAL VARIABLES **********************************/

// regulation's specific variables
extern volatile struct RegulSignals RegulSig;
extern volatile struct PID Pid[NB_ROTOR];
extern volatile struct FilterSignals FilterSig[NB_ROTOR];
extern volatile struct FilterSignals FilterMeas[NB_ROTOR];

extern volatile double Y_LookUpTable_FF[SIZE_LOOKUP_FF];	// [us] the time_up PWM i corresponding to the open-loop freq i
extern volatile double X_LookUpTable_FF[SIZE_LOOKUP_FF];  // [us] the freq i corresponding to the open-loop PWM i
extern volatile double a_LookUpTable_FF[SIZE_LOOKUP_FF-1];
extern volatile double b_LookUpTable_FF[SIZE_LOOKUP_FF-1];

extern float dispRotorSpeed[NB_ROTOR];
extern uint32_t StepTime;
/******************************** END GLOBALE VARIABLES ********************************/


/******************************** STRUCTURE ********************************/
struct RegulSignals
{
	float 	RotorSpeed_SetPoint[NB_ROTOR];				// [rad/sec]
	float 	Measured_RotorSpeed[NB_ROTOR];		    	// [rad/sec]
	float 	Previous_Measured_RotorSpeed[NB_ROTOR];		// [rad/sec]
	float 	Previous2_Measured_RotorSpeed[NB_ROTOR];	// [rad/sec]
	float 	TimeUp_PWM_ESC[NB_ROTOR];					// [us] between MINCOMMAND and MAX_TIMEUP_PWM
	uint8_t	RotorStatus[NB_ROTOR];				// motors armed or not
	uint8_t	ESCStatus[NB_ROTOR];				// ESCs armed or not
	uint8_t	RegulState;								// CLOSED_LOOP, OPEN_LOOP, IDLE
};

struct PID
{
	float kp;
	float ki;
	float kaw;
	float G_SatMax;
	float G_SatMin;
	float I_SatMax;
	float I_SatMin;
	float Integrator;
};

struct FilterSignals
{
	float	RotorSpeed_FilteredSetPoint;		// [rad/sec]
	float 	InputHistory[FILTER_ORDER];			// [rad/sec]
	float 	FilterStateHistory[FILTER_ORDER];	// [rad/sec]
	float 	a0_order1;
	float 	a1_order1;
	float 	a0_order2;
	float 	a1_order2;
	float	a2_order2;
};
/******************************** END STRUCTURE ********************************/


/*********************************** FUNCTIONS PROTOTYPES **********************************/

void initRegul(void);
void StartESC(uint8_t ESCNum);
void StartRotor(uint8_t RotorNum);
void StopRotor(uint8_t RotorNum);

void RegulRotorSpeed(uint8_t rotor_num);
void InitSetPointFilter(void);
void InitMeasureFilter(void);
void SetPointFilter(uint8_t rotor_num);
void InitLookUpTable(void);
float ComputeFeedForward(float FreqSetPoint);

float us2freq_table(float SetPoint_us);
extern float Saturate(float Input, float Min, float Max);

/******************************** END FUNCTIONS PROTOTYPES ********************************/

#endif