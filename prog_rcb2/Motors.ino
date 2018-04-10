/*
This file is part of QuadMorphing project
Copyright (C) 2018 - Institute of Movement Sciences (Marseille, FRANCE)

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "Def.h"

extern volatile int16_t SetPoint_us[NB_ROTOR];

void initMotors() 
{
	// Default frequency with teensy 3.1 : 488.28Hz (analogWriteFrequency(pin,frequency))
	// Timer used for pin 20,21,22,23 : FTM0 (default: 488.28Hz)

	// mark all PWM pins as Output 
	pinMode(PIN_M1,OUTPUT);
	pinMode(PIN_M2,OUTPUT);
	pinMode(PIN_M3,OUTPUT);
	pinMode(PIN_M4,OUTPUT);
	
	// Resolution config
	analogWriteResolution(BITS_PWM_RESOLUTION); 

	// To initialize ESCs  
	WriteAllMotors(MINCOMMAND);
	delay(1000);
	timer_refresh_OC.begin(refresh_OC,OC_UPDATE);
}

void WriteMotors()
{ 
	for (uint8_t i =0;i<NB_ROTOR;i++)
		SetPoint_us[i] = motor[i];
}

void WriteAllMotors(int16_t mc) 
{   
	for (uint8_t i =0;i<NB_ROTOR;i++) 
		SetPoint_us[i] = mc;
}

void refresh_OC(void)		// Update PWM timer_refresh_OC interrupt
{
	cli();
		// MOTOR 1 (front)
		analogWrite(PIN_M1,(float) RegulSig.TimeUp_PWM_ESC[0]/MAXTHROTTLE*PWM_CONS_MAX);

		// MOTOR 2 (left)
		analogWrite(PIN_M2,((float) RegulSig.TimeUp_PWM_ESC[1]/MAXTHROTTLE)*PWM_CONS_MAX);

		// MOTOR 3 (rear)
		analogWrite(PIN_M3,(float) RegulSig.TimeUp_PWM_ESC[2]/MAXTHROTTLE*PWM_CONS_MAX);

		// MOTOR 4 (right)
		analogWrite(PIN_M4,(float) RegulSig.TimeUp_PWM_ESC[3]/MAXTHROTTLE*PWM_CONS_MAX);
	sei();
}