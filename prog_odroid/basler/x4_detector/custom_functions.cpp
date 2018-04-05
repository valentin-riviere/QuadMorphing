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
#include "custom_functions.h"

/**
 * @function cst_angle
 */
double cst_angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;

    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

/**
 * @function rect_extern_contour
 */
Rect rect_extern_contour(const Square & square)
{
	uint16_t min_x = square[0].x, max_x = square[0].x, min_y = square[0].y, max_y = square[0].y;

	for (uint8_t i = 1 ; i < square.size() ; i++)
	{
		if (square[i].x < min_x)
			min_x = square[i].x;
		if (square[i].y < min_y)
			min_y = square[i].y;
		if (square[i].x > max_x)
			max_x = square[i].x;
		if (square[i].y > max_y)
			max_y = square[i].y;
	}
	
	Rect rect_ext(min_x,min_y,max_x-min_x,max_y-min_y);
	return rect_ext;
}

/**
 * @function sub_angles_from_square
 */
void sub_angles_from_square(const Square & square, float * sub_angles, const Point & roi_pos, const unsigned int & width, const unsigned int & height, const float * FOV_div2)
{
	// p[4] = Corners bottomleft, topleft, topright, bottomright
	Point p[4] = {Point(square[0]), Point(square[1]), Point(square[2]), Point(square[3])};
	Point tmp;
	// c[4] = Centers of segments left, up, right, down
	Point c[4];

	// Sort with x increasing
	for (uint8_t i = 0 ; i < square.size() ; i++)
	{
		for (uint8_t j = i+1 ; j < square.size() ; j++)
		{
			if (p[i].x > p[j].x)
			{
				tmp = p[i];
				p[i] = p[j];
				p[j] = tmp;
			}
		}
	}

	// Sort SW->NW->NE->SE
	if (p[0].y < p[1].y)
	{
		tmp = p[0];
		p[0] = p[1];
		p[1] = tmp;
	}
	if (p[2].y > p[3].y)
	{
		tmp = p[2];
		p[2] = p[3];
		p[3] = tmp;
	}

	// Segments center
	for (uint8_t i = 0 ; i < square.size() ; i++)
	{
		c[i].x = (p[i].x+p[(i+1)%4].x)/2;
		c[i].y = (p[i].y+p[(i+1)%4].y)/2;
	}

	// Subtented angles
	sub_angles[LEFT] = atan((2.0 * ((float)(c[0].x+roi_pos.x)/width) - 1.0)*tan(FOV_div2[0]));
	sub_angles[RIGHT] = atan((2.0 * ((float)(c[2].x+roi_pos.x)/width) - 1.0)*tan(FOV_div2[0]));
	sub_angles[UP] = atan((-2.0 * ((float)(c[1].y+roi_pos.y)/height) + 1.0)*tan(FOV_div2[1]));
	sub_angles[DOWN] = atan((-2.0 * ((float)(c[3].y+roi_pos.y)/height) + 1.0)*tan(FOV_div2[1]));
}
