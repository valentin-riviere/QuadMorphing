/*
This file is part of QuadMorphing project
Copyright (C) 2018 - Institute of Movement Sciences (Marseille, FRANCE)

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "Tachys.h"

volatile uint16_t t1_prev, t2_prev, t3_prev, t4_prev;	// if uint16_t f€[16Hz 500kHz] <=> T€[10us 65535us]

extern volatile uint32_t TachyRisingPeriod[NB_ROTOR];
extern volatile uint16_t TachyNbOverFlow[NB_ROTOR];
extern IntervalTimer timer_overflow_IC;

void isr_1() // TACHY 1
{
#ifdef DEBUG_TACHYS
	Serial.println("Tick tachy 1");
#endif
cli();
	uint32_t cTime = micros();
	static uint32_t eTime = 0;

	TachyRisingPeriod[ROTOR_1] = cTime-eTime;
	eTime = cTime;
	
	TachyNbOverFlow[ROTOR_1] = 0;
sei();
}

void isr_2() // TACHY 2
{
#ifdef DEBUG_TACHYS
	Serial.println("Tick tachy 2");
#endif
cli();
	uint32_t cTime = micros();
	static uint32_t eTime = 0;

	TachyRisingPeriod[ROTOR_2] = cTime-eTime;
	eTime = cTime;
	
	TachyNbOverFlow[ROTOR_2] = 0;
sei();
}

void isr_3() // TACHY 3
{
#ifdef DEBUG_TACHYS
	Serial.println("Tick tachy 3");
#endif
cli();
	uint32_t cTime = micros();
	static uint32_t eTime = 0;

	TachyRisingPeriod[ROTOR_3] = cTime-eTime;
	eTime = cTime;
	
	TachyNbOverFlow[ROTOR_3] = 0;
sei();
}

void isr_4() // TACHY 4
{
#ifdef DEBUG_TACHYS
	Serial.println("Tick tachy 4");
#endif
cli();
	uint32_t cTime = micros();
	static uint32_t eTime = 0;

	TachyRisingPeriod[ROTOR_4] = cTime-eTime;
	eTime = cTime;
	
	TachyNbOverFlow[ROTOR_4] = 0;
sei();
}

void overflowIC(void)
{
#ifdef DEBUG_TACHYS
	Serial.println("Overflow interrupt");
#endif
cli();
	uint8_t i;
	
	for(i=0; i<NB_ROTOR; i++)
	{
		TachyNbOverFlow[i]++;
		if (TachyNbOverFlow[i] > 2)
		{
			TachyRisingPeriod[i] += 65536LU;
		}
	}
sei();
}

void setupTachys(void)
{
	pinMode(PIN_T1, INPUT);pinMode(PIN_T2, INPUT);pinMode(PIN_T3, INPUT);pinMode(PIN_T4, INPUT);
	attachInterrupt(digitalPinToInterrupt(PIN_T1), isr_1, CHANGE);
	attachInterrupt(digitalPinToInterrupt(PIN_T2), isr_2, CHANGE);
	attachInterrupt(digitalPinToInterrupt(PIN_T3), isr_3, CHANGE);
	attachInterrupt(digitalPinToInterrupt(PIN_T4), isr_4, CHANGE);
	
	timer_overflow_IC.begin(overflowIC,65536);
}