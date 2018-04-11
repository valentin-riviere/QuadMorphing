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
 * @function sort_square_points
 */
void sort_square_points(Square & square)
{
	vector<Point*> p; p.push_back(&(square[0])); p.push_back(&(square[1])); p.push_back(&(square[2])); p.push_back(&(square[3]));
	Point tmp;

	// Sort with x increasing
	for (uint8_t i = 0 ; i < square.size() ; i++)
	{
		for (uint8_t j = i+1 ; j < square.size() ; j++)
		{
			if (p[i]->x > p[j]->x)
			{
				tmp = *p[i];
				*p[i] = *p[j];
				*p[j] = tmp;
			}
		}
	}

	// Sort BL->TL->TR->BR
	if (p[0]->y < p[1]->y)
	{
		tmp = *p[0];
		*p[0] = *p[1];
		*p[1] = tmp;
	}
	if (p[2]->y > p[3]->y)
	{
		tmp = *p[2];
		*p[2] = *p[3];
		*p[3] = tmp;
	}
}

/**
 * @function sub_angles_from_square
 */
void sub_angles_from_square(const Square & square, float * sub_angles, const Point & roi_pos, const unsigned int & width, const unsigned int & height, const float * FOV_div2)
{
	// Cpy of square
	Square s(square);
	// c[4] = Centers of segments left, up, right, down
	Point c[4];

	sort_square_points(s);

	// Segments center
	for (uint8_t i = 0 ; i < s.size() ; i++)
	{
		c[i].x = (s[i].x+s[(i+1)%4].x)/2;
		c[i].y = (s[i].y+s[(i+1)%4].y)/2;
	}

	// Subtented angles
	sub_angles[LEFT] = atan((2.0 * ((float)(c[0].x+roi_pos.x)/width) - 1.0)*tan(FOV_div2[0]));
	sub_angles[RIGHT] = atan((2.0 * ((float)(c[2].x+roi_pos.x)/width) - 1.0)*tan(FOV_div2[0]));
	sub_angles[UP] = atan((-2.0 * ((float)(c[1].y+roi_pos.y)/height) + 1.0)*tan(FOV_div2[1]));
	sub_angles[DOWN] = atan((-2.0 * ((float)(c[3].y+roi_pos.y)/height) + 1.0)*tan(FOV_div2[1]));
}

/**
 * @function select_center_square
 */
void select_center_square(const vector<Square> & squares, Square & sel_square, const unsigned int & width, const unsigned int & height)
{
	const uint32_t r2_min = width*width+height*height;
	for( size_t i = 0; i < squares.size(); i++ )
	{
		uint32_t x = (squares[i][0].x + squares[i][1].x + squares[i][2].x + squares[i][3].x)/4 - width;
		uint32_t y = (squares[i][0].y + squares[i][1].y + squares[i][2].y + squares[i][3].y)/4 - height;
		if (x*x+y*y < r2_min)
			sel_square = squares[i];
	}
}

/**
 * @function select_min_square
 */
void select_min_square(const vector<Square> & squares, Square & sel_square)
{
	uint32_t area_min = numeric_limits<uint32_t>::max();
	for( size_t i = 0; i < squares.size(); i++ )
	{
		Rect ext_contour = rect_extern_contour(squares[i]);
		uint32_t area = ext_contour.width * ext_contour.height;

		if (area < area_min)
		{
			area_min = area;
			sel_square = squares[i];
		}
	}
}

/**
 * @function select_square
 */
int8_t select_square(const vector<Square> & squares, Square & sel_square, const uint16_t & thresh_diff2, const float & thresh_ratio2)
{
	int8_t out(-1); // Return value depends of passed step

	vector<vector<Square> > squares_2;	// Vector of 2 associated squares
	vector<Square> squares_out;
	vector<Point> p;				// Centers of squares
	vector<float> areas;			// Vector of squares areas

	// Clear selected square
	sel_square.clear();

#ifdef DEBUG_SEL_SQUARE
	Mat img(800,600,CV_8UC1,Scalar(0));
	print_squares(img,squares,"Before selection",0);
#endif

	// Centers
	for( size_t i = 0; i < squares.size(); i++ )
	{
		Point c;
		c.x = (squares[i][0].x + squares[i][1].x + squares[i][2].x + squares[i][3].x)/4;
		c.y = (squares[i][0].y + squares[i][1].y + squares[i][2].y + squares[i][3].y)/4;

		p.push_back(c);
		out = 0;
	}

	// Distance^2 between squares centers
	for( size_t i = 0; i < p.size(); i++ )
	{
		for( size_t j = i+1; j < p.size(); j++ )
		{
			uint32_t dist2 = (p[j].x-p[i].x)*(p[j].x-p[i].x)+(p[j].y-p[i].y)*(p[j].y-p[i].y);
			out = 1;
		#ifdef DEBUG_SEL_SQUARE
			cout << "Center distance error : " <<  dist2 << endl;
		#endif
			if (dist2 < thresh_diff2)	// Keep pair of squaresif dist^2 < thresh
			{
				vector<Square> s_2; s_2.push_back(squares[i]); s_2.push_back(squares[j]);
				squares_2.push_back(s_2);
				out = 2;
			}
		}
	}

	// Check ratio between pair of squares
	for( size_t i = 0; i < squares_2.size(); i++ )
	{
		// Sort square points
		sort_square_points(squares_2[i][0]);
		sort_square_points(squares_2[i][1]);

		Square s1 = squares_2[i][0], s2 = squares_2[i][1];
		float w1, w2, h1, h2, r1, r2;	// Ratio^2 between rect width and height
		w1 = (float) (s1[2].x-s1[1].x)*(s1[2].x-s1[1].x)+(s1[2].y-s1[1].y)*(s1[2].y-s1[1].y);
		w2 = (float) (s2[2].x-s2[1].x)*(s2[2].x-s2[1].x)+(s2[2].y-s2[1].y)*(s2[2].y-s2[1].y);
		h1 = (float) (s1[1].x-s1[0].x)*(s1[1].x-s1[0].x)+(s1[1].y-s1[0].y)*(s1[1].y-s1[0].y);
		h2 = (float) (s2[1].x-s2[0].x)*(s2[1].x-s2[0].x)+(s2[1].y-s2[0].y)*(s2[1].y-s2[0].y);
		r1 = w1/h1;
		r2 = w2/h2;

	#ifdef DEBUG_SEL_SQUARE
		cout << "Ratio error : " <<  abs(r1 - r2) << endl;
	#endif

		if ( abs(r1 - r2) < thresh_ratio2 ) // If diff between ratio^2 < thresh_ratio2 => push pair of squares to list of squares out
		{
			squares_out.push_back(s1);
			squares_out.push_back(s2);
			areas.push_back(w1*h1);
			areas.push_back(w2*h2);
			out = 3;
		}
	}

#ifdef DEBUG_SEL_SQUARE
	print_squares(img,squares_out,"After area check",0);
#endif

	// Take the smallest square
	float area_min = numeric_limits<float>::max();
	for( size_t i = 0; i < areas.size(); i++ )
	{
#ifdef DEBUG_SEL_SQUARE
		cout << "Areas : " <<  areas[i] << endl;
		print_squares(img,squares_out[i],"area processing",0);
#endif
		if (areas[i] < area_min)
		{
			area_min = areas[i];
			sel_square = squares_out[i];
		}
	}

#ifdef DEBUG_SEL_SQUARE
	print_squares(img,sel_square,"Final selection",0);
#endif

	return out;
}

/**
 * @function select_center_square
 */
void update_roi(Rect & ROI, const Square & sel_square, const float * roi_offsets, const unsigned int & width, const unsigned int & height)
{
	Rect ext_contour = rect_extern_contour(sel_square);
	uint16_t tmp, o_w = ext_contour.width*roi_offsets[0]/100, o_h = ext_contour.height*roi_offsets[0]/100;
	tmp = ROI.x + ext_contour.x - o_w;
	if (tmp < 0 || tmp > width) ROI.x = 0;
		else ROI.x = tmp;
	tmp = ROI.y + ext_contour.y - o_h;
	if (tmp < 0 || tmp > height) ROI.y = 0;
		else ROI.y = tmp;
	tmp = ext_contour.width + 2*o_w;
	if (tmp + ROI.x > width) ROI.width = width - ROI.x;
		else ROI.width = tmp;
	tmp = ext_contour.height + 2*o_h;
	if (tmp + ROI.y > height) ROI.height = height - ROI.y;
		else ROI.height = tmp;
}
