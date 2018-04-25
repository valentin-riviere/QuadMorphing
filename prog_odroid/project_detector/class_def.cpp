/*
This file is part of QuadMorphing project
Copyright (C) 2018 - Institute of Movement Sciences (Marseille, FRANCE)
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "class_def.h"

void Stream_in::print() const
{
	cout 	<< t_poll << '\n'
			<< (uint16_t) max_num_buffer << '\n'
			<< width << '\n'
			<< height << '\n'
			<< offset_x << '\n'
			<< offset_y << '\n'
			<< (uint16_t) binning << '\n'
			<< expo_auto_max << '\n'
			<< grab_time_out << '\n'
			<< fov[0] << '\n'
			<< fov[1] << '\n'
			<< (uint16_t) max_no_detect << '\n'
			<< (uint16_t) size_blur << '\n'
			<< (uint16_t) bin_square << '\n'
			<< k_square << '\n'
			<< area_square << '\n'
			<< cos_square << '\n'
			<< thresh_diff2 << '\n'
			<< thresh_ratio2 << '\n';
}


void Stream_in::convert_from_buffer(const uint8_t * read_buffer)
{
	uint8_2_uint16(read_buffer,&t_poll,1);
	max_num_buffer = *(read_buffer+2);
	uint8_2_uint16(read_buffer+3,&width,1);
	uint8_2_uint16(read_buffer+5,&height,1);
	uint8_2_uint16(read_buffer+7,&offset_x,1);
	uint8_2_uint16(read_buffer+9,&offset_y,1);
	binning = *(read_buffer+11);
	uint8_2_uint16(read_buffer+12,&expo_auto_max,1);
	uint8_2_uint16(read_buffer+14,&grab_time_out,1);
	uint8_2_float(read_buffer+16,fov,2);
	max_no_detect = *(read_buffer+24);
	size_blur = *(read_buffer+25);
	bin_square = *(read_buffer+26);
	uint8_2_float(read_buffer+27,&k_square,1);
	uint8_2_uint16(read_buffer+31,&area_square,1);
	uint8_2_float(read_buffer+33,&cos_square,1);
	uint8_2_uint16(read_buffer+37,&thresh_diff2,1);
	uint8_2_float(read_buffer+39,&thresh_ratio2,1);
}

void Stream_out::print() const
{
	cout 	<< sub_angles[0] << '\n'
			<< sub_angles[1] << '\n'
			<< sub_angles[2] << '\n'
			<< sub_angles[3] << '\n'
			<< time_frame << '\n'
			<< (uint16_t) no_detect << '\n'
			<< (uint16_t) init_detect << '\n';
}

void Stream_out::to_buffer(uint8_t * write_buffer) const
{
	uint8_t array8[16];
	float_2_uint8(sub_angles, write_buffer,4);
	uint64_2_uint8(&time_frame,write_buffer+16);
	memcpy(write_buffer+24,&no_detect,1);
	memcpy(write_buffer+25,&init_detect,1);
}

