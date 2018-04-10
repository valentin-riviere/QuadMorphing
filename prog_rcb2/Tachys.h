/*
This file is part of QuadMorphing project
Copyright (C) 2018 - Institute of Movement Sciences (Marseille, FRANCE)

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef __TACHYS_H__
#define __TACHYS_H__

#include <Arduino.h>
#include "Def.h"

/*********************************** GLOBAL VARIABLES **********************************/

extern volatile uint16_t t1_prev, t2_prev, t3_prev, t4_prev;	// if uint16_t f€[16Hz 500kHz] <=> T€[10us 65535us]

/*********************************** END GLOBAL VARIABLES **********************************/

/*********************************** FUNCTIONS PROTOTYPES **********************************/

void setupTachys(void);
void overflowIC(void);
void isr_1(void);
void isr_2(void);
void isr_3(void);
void isr_4(void);

/******************************** END FUNCTIONS PROTOTYPES ********************************/

#endif