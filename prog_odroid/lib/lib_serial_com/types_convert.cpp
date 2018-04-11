/*
This file is part of QuadMorphing project
Copyright (C) 2018 - Institute of Movement Sciences (Marseille, FRANCE)

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "types_convert.h"

void uint8_2_uint16(const uint8_t * array8, uint16_t * array16, const uint8_t N)
{
#ifdef L_ENDIAN
	for (uint8_t i = 0 ; i < N ; i++)
	{
		array16[i] = ( (uint16_t) array8[2*i] << 8) + array8[2*i+1];
	}
#else
	memcpy(array16,array8,2*N);
#endif
}

void uint8_2_int16(const uint8_t * array8, int16_t * array16, const uint8_t N)
{
#ifdef L_ENDIAN
	for (uint8_t i = 0 ; i < N ; i++)
	{
		array16[i] = ( (uint16_t) array8[2*i] << 8) + array8[2*i+1];
	}
#else
	memcpy(array16,array8,2*N);
#endif
}

void uint8_2_float(const uint8_t * array8, float * float32, const uint8_t N)
{
#ifdef L_ENDIAN
	uint32_t *array32 = (uint32_t*) malloc(N*sizeof(uint32_t));
	
	for (uint8_t i = 0 ; i < N ; i++)
	{
		array32[i] = ( (uint32_t) array8[4*i] << 24) + ( (uint32_t) array8[4*i+1] << 16) + ( (uint32_t) array8[4*i+2] << 8) + (uint32_t) array8[4*i+3];
		//printf("%x\n", float32[i]);
	}

	memcpy(float32,array32,4*N);
	free(array32);
#else
	memcpy(float32,array8,4*N);
#endif
}

void uint16_2_uint8(const uint16_t * array16, uint8_t * array8, const uint8_t N)
{
#ifdef L_ENDIAN
	memcpy(array8,array16,2*N);

	for (uint8_t i = 0 ; i < N ; i++)
	{
		uint16_t tmp = ((uint32_t) array8[2*i] << 8) + (uint32_t) array8[2*i+1];
		memcpy(&array8[2*i],&tmp,2);	
		//printf("%x %x\n", array8[2*i], array8[2*i+1]);
	}
	
#else
	memcpy(array8,array16,2*N);
#endif
}

void int16_2_uint8(const int16_t * array16, uint8_t * array8, const uint8_t N)
{
#ifdef L_ENDIAN
	memcpy(array8,array16,2*N);

	for (uint8_t i = 0 ; i < N ; i++)
	{
		uint16_t tmp = ((uint32_t) array8[2*i] << 8) + (uint32_t) array8[2*i+1];
		memcpy(&array8[2*i],&tmp,2);	
		//printf("%x %x\n", array8[2*i], array8[2*i+1]);
	}
	
#else
	memcpy(array8,array16,2*N);
#endif
}

void float_2_uint8(const float * float32, uint8_t * array8, const uint8_t N)
{
#ifdef L_ENDIAN
	memcpy(array8,float32,4*N);

	for (uint8_t i = 0 ; i < N ; i++)
	{
		uint32_t tmp = ((uint32_t) array8[4*i] << 24) + ((uint32_t) array8[4*i+1] << 16) + ((uint32_t) array8[4*i+2] << 8) + (uint32_t) array8[4*i+3];
		memcpy(&array8[4*i],&tmp,4);	
		//printf("%x %x %x %x\n", array8[4*i], array8[4*i+1], array8[4*i+2], array8[4*i+3]);
	}
	
#else
	memcpy(array8,float32,4*N);
#endif
}
