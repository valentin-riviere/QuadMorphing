/*
This file is part of QuadMorphing project
Copyright (C) 2018 - Institute of Movement Sciences (Marseille, FRANCE)

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "Def.h"

/**************************************************************************************/
/***************             Global RX related variables           ********************/
/**************************************************************************************/

//RAW RC values will be store here
volatile uint16_t rcValue[8] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]


/**************************************************************************************/
/***************                   RX Pin Setup                    ********************/
/**************************************************************************************/
void configureReceiver()
{
	pinMode(THROTTLEPIN,INPUT);
	pinMode(AUX1PIN,INPUT);
	pinMode(AUX2PIN,INPUT);
	pinMode(YAWPIN,INPUT);
	pinMode(PITCHPIN,INPUT);
	pinMode(ROLLPIN,INPUT);
	attachInterrupt(digitalPinToInterrupt(THROTTLEPIN), isr_throttle, CHANGE);
	attachInterrupt(digitalPinToInterrupt(AUX1PIN), isr_aux1, CHANGE);
	attachInterrupt(digitalPinToInterrupt(AUX2PIN), isr_aux2, CHANGE);
	attachInterrupt(digitalPinToInterrupt(YAWPIN), isr_yaw, CHANGE);
	attachInterrupt(digitalPinToInterrupt(PITCHPIN), isr_pitch, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ROLLPIN), isr_roll, CHANGE);
}

/**************************************************************************************/
/***************               Standard RX Pins reading            ********************/
/**************************************************************************************/
void isr_throttle() // Throttle interruption
{
	uint16_t cTime = micros();

	static uint16_t eTime;
	uint16_t dTime;

	if (digitalReadFast(THROTTLEPIN) != 0)
		eTime = cTime;
	else
	{
		dTime = cTime-eTime; if (900<dTime && dTime<2200) rcValue[THROTTLE] = dTime;
	}
}

void isr_aux1() // Aux1 interruption
{
	uint16_t cTime = micros();

	static uint16_t eTime;
	uint16_t dTime;

	if (digitalReadFast(AUX1PIN) != 0)
		eTime = cTime;
	else
	{
		dTime = cTime-eTime; if (900<dTime && dTime<2200) rcValue[AUX1] = dTime;
	}
}

void isr_aux2() // Aux2 interruption
{
	uint16_t cTime = micros();

	static uint16_t eTime;
	uint16_t dTime;

	if (digitalReadFast(AUX2PIN) != 0)
		eTime = cTime;
	else
	{
		dTime = cTime-eTime; if (900<dTime && dTime<2200) rcValue[AUX2] = dTime;
	}
}

void isr_yaw() // Yaw interruption
{
	uint16_t cTime = micros();

	static uint16_t eTime;
	uint16_t dTime;

	if (digitalReadFast(YAWPIN) != 0)
		eTime = cTime;
	else
	{
		dTime = cTime-eTime; if (900<dTime && dTime<2200) rcValue[YAW] = dTime;
	}
}

void isr_pitch() // Pitch interruption
{
	uint16_t cTime = micros();

	static uint16_t eTime;
	uint16_t dTime;

	if (digitalReadFast(PITCHPIN) != 0)
		eTime = cTime;
	else
	{
		dTime = cTime-eTime; if (900<dTime && dTime<2200) rcValue[PITCH] = dTime;
	}
}

void isr_roll() // Roll interruption
{
	uint16_t cTime = micros();

	static uint16_t eTime;
	uint16_t dTime;

	if (digitalReadFast(ROLLPIN) != 0)
		eTime = cTime;
	else
	{
		dTime = cTime-eTime; if (900<dTime && dTime<2200) rcValue[ROLL] = dTime;
	}
}

/**************************************************************************************/
/***************          combine and sort the RX Datas            ********************/
/**************************************************************************************/
uint16_t readRawRC(uint8_t chan)
{
	uint16_t data;
	
cli();
	data = rcValue[chan]; // Let's copy the data Atomically
sei();
	
	return data;
}
/**************************************************************************************/
/***************          compute and Filter the RX data           ********************/
/**************************************************************************************/
void computeRC()
{
	static int16_t rcData4Values[8][4], rcDataMean[8];
	static uint8_t rc4ValuesIndex = 0;
	uint8_t chan,a;
	
	rc4ValuesIndex++;
	for (chan = 0; chan < 8; chan++)
	{
		rcData4Values[chan][rc4ValuesIndex%4] = readRawRC(chan);
		rcDataMean[chan] = 0;
		for (a=0;a<4;a++) rcDataMean[chan] += rcData4Values[chan][a];
		rcDataMean[chan]= (rcDataMean[chan]+2)/4;
		if ( rcDataMean[chan] < rcData[chan] -3)  rcData[chan] = rcDataMean[chan]+2;
		if ( rcDataMean[chan] > rcData[chan] +3)  rcData[chan] = rcDataMean[chan]-2;
	}	
}

void CheckArmMotors(void)
{
	static uint8_t rcDelayCommand = 0; // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors

	if (rcData[THROTTLE] < MINCHECK) // throttle min, yaw left
	{
		if (rcData[YAW] > MAXCHECK && !MotorsArmed) 
		{
			rcDelayCommand++;
			if (rcDelayCommand == 20) //0.4s = 20*0.02s
			{
				WriteAllMotors(MINTHROTTLE);
				// motor on
				MotorsArmed = 1;
			}
		}
		else if(rcData[YAW] < MINCHECK && MotorsArmed) // throttle min, yaw right
		{
			rcDelayCommand++;
			if (rcDelayCommand == 20) //0.4s = 20*0.02
			{
				WriteAllMotors(MINCOMMAND);
				// motor off
				MotorsArmed = 0;
			}
		}
		else
		{
			rcDelayCommand = 0;
		}
	}
}

void RX_loop()
{
	computeRC();
	
	if (rcData[AUX1] > 1500 )
	{
		ManualMode = 1;
	}
	else
	{
		ManualMode = 0;
	}

	// Arm Motor Procedure
	CheckArmMotors();
}
