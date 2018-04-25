/*
This file is part of QuadMorphing project
Copyright (C) 2018 - Institute of Movement Sciences (Marseille, FRANCE)

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef TYPES_CONVERT_H
#define TYPES_CONVERT_H

// Check ENDIANESS
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	#define L_ENDIAN
#else
	#define B_ENDIAN
#endif

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <stdint.h>

using namespace std;

// WARNING : Saving var depends of BYTE_ORDER of the arch (see flags above)

// Convert uint8_t array to N uint16_t/int16_t (Big Endian array uint8 : 1st byte = MSB)
void uint8_2_uint16(const uint8_t * array8, uint16_t * array16, const uint8_t N = 1);
void uint8_2_int16(const uint8_t * array8, int16_t * array16, const uint8_t N = 1);

// Convert uint8_t array to N float (Big Endian array uint8 : 1st byte = MSB)
void uint8_2_float(const uint8_t * array8, float * float32, const uint8_t N = 1);

// Convert N uint16_t/int16_t array to 2*N uint8_t (Big Endian array uint8 : 1st byte = MSB)
void uint16_2_uint8(const uint16_t * array16, uint8_t * array8, const uint8_t N = 1);
void int16_2_uint8(const int16_t * array16, uint8_t * array8, const uint8_t N = 1);

// Convert N uint64_t array to 8*N uint8_t (Big Endian array uint8 : 1st byte = MSB)
void uint64_2_uint8(const uint64_t * array64, uint8_t * array8, const uint8_t N = 1);

// Convert N float array to 4*N uint8_t (Big Endian array uint8 : 1st byte = MSB)
void float_2_uint8(const float * float32, uint8_t * array8, const uint8_t N = 1);

#endif
