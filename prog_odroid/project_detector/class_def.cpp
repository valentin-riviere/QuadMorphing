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
			<< max_num_buffer << '\n'
			<< width << '\n'
			<< height << '\n'
			<< offset_x << '\n'
			<< offset_y << '\n'
			<< binning << '\n'
			<< expo_auto_max << '\n'
			<< grab_time_out << '\n'
			<< fov[0] << '\n'
			<< fov[1] << '\n'
			<< max_no_detect << '\n'
			<< size_blur << '\n'
			<< bin_square << '\n'
			<< k_square << '\n'
			<< area_square << '\n'
			<< cos_square << '\n'
			<< thresh_diff2 << '\n'
			<< thresh_ratio2 << '\n';
}


void Stream_in::convert_from_buffer(const uint8_t * read_buffer)
{
	uint8_2_uint16(read_buffer,&t_poll,1);
	uint8_2_uint16(read_buffer+2,&max_num_buffer,1);
	uint8_2_uint16(read_buffer+4,&width,1);
	uint8_2_uint16(read_buffer+6,&height,1);
	uint8_2_uint16(read_buffer+8,&offset_x,1);
	uint8_2_uint16(read_buffer+10,&offset_y,1);
	uint8_2_uint16(read_buffer+12,&binning,1);
	uint8_2_uint16(read_buffer+14,&expo_auto_max,1);
	uint8_2_uint16(read_buffer+16,&grab_time_out,1);
	uint8_2_float(read_buffer+18,fov,2);
	uint8_2_uint16(read_buffer+26,&max_no_detect,1);
	uint8_2_uint16(read_buffer+28,&size_blur,1);
	uint8_2_uint16(read_buffer+30,&bin_square,1);
	uint8_2_float(read_buffer+32,&k_square,1);
	uint8_2_uint16(read_buffer+36,&area_square,1);
	uint8_2_float(read_buffer+38,&cos_square,1);
	uint8_2_uint16(read_buffer+42,&thresh_diff2,1);
	uint8_2_float(read_buffer+44,&thresh_ratio2,1);
}

void Stream_out::print() const
{
	cout 	<< sub_angles[0] << '\n'
			<< sub_angles[1] << '\n'
			<< sub_angles[2] << '\n'
			<< sub_angles[3] << '\n'
			<< time_frame << '\n'
			<< no_detect << '\n'
			<< init_detect << '\n';
}

void Stream_out::to_buffer(uint8_t * write_buffer) const
{
	uint8_t array8[16];
	float_2_uint8(sub_angles, write_buffer,4);
	uint64_2_uint8(&time_frame,write_buffer+16);
	uint16_2_uint8(&no_detect,write_buffer+24);
	uint16_2_uint8(&init_detect,write_buffer+26);
}

