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
#ifndef DEF_H
#define DEF_H

#include <iostream>

using namespace std;

// Data stream in
class Stream_in
{
	public:
		uint16_t t_poll;
		uint8_t max_num_buffer;
		uint16_t width, height;
		uint16_t offset_x, offset_y;
		uint8_t binning;
		uint16_t expo_auto_max;
		uint16_t grab_time_out;
		float fov[2];
		uint8_t max_no_detect;
		uint8_t size_blur;
		uint8_t bin_square;
		float k_square;
		uint16_t area_square;
		float cos_square;
		uint16_t thresh_diff2;
		float thresh_ratio2;

		void print();
};

// Data stream out
class Stream_out
{
	public:
		float sub_angles[4];
		uint64_t time_frame;
		uint8_t no_detect;
		uint8_t init_detect;
};

#endif
