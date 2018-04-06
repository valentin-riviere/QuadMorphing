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
#ifndef CUSTOM_FUNCTIONS_H
#define CUSTOM_FUNCTIONS_H

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <math.h>
#include <string.h>

#define LEFT	0
#define UP		1
#define RIGHT	2
#define DOWN	3

// Debug flags
//#define DEBUG_SEL_SQUARE // Debug square selection

using namespace cv;
using namespace std;

typedef vector<Point> Square;

// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
double cst_angle( Point pt1, Point pt2, Point pt0 );

// Find rectangular contour of a square
Rect rect_extern_contour(const Square & square);

// Find index of the top left point of a square
uint16_t tl_square(const Square & square);

// Sort square BL->TL->TR->BR
void sort_square_points(Square & square);

// Draw squares
void draw_squares(const Mat & src, const vector<Square> & squares, Mat & dst, const Scalar & color = Scalar(0,255,0));
void draw_squares(const Mat & src, const vector<Square> & squares, Mat & dst, const Rect & roi, const Scalar & color = Scalar(0,255,0));
void draw_squares(const Mat & src, const Square & square, Mat & dst, const Scalar & color = Scalar(0,255,0));
void draw_squares(const Mat & src, const Square & square, Mat & dst, const Rect & roi, const Scalar & color = Scalar(0,255,0));

// Print squares
void print_squares(const Mat & src, const vector<Square> & squares, const string & win_name, const uint16_t & t_wait = 10, const Scalar & color = Scalar(0,255,0));
void print_squares(const Mat & src, const Square & square, const string & win_name, const uint16_t & t_wait = 10, const Scalar & color = Scalar(0,255,0));

// Print image
void print_img(const Mat & src, const string & win_name, const uint16_t & t_wait = 10);

// Select the more centered square
void select_center_square(const vector<Square> & squares, Square & sel_square, const unsigned int & width, const unsigned int & height);

// Select the smallest square
void select_min_square(const vector<Square> & squares, Square & sel_square);

// Select the smallest square of the two squares with the same ratio (ratio^2 < thresh_ratio2) and same center (diff^2 < thresh_diff^2)
void select_square(const vector<Square> & squares, Square & sel_square, const uint16_t & thresh_diff2, const float & thresh_ratio2);

// Update ROI
void update_roi(Rect & ROI, const Square & sel_square, const uint16_t * roi_offsets, const unsigned int & width, const unsigned int & height);

// Find subtented angles from square (left, up, right, down : origin in the center of the image and positive on right and top of image)
void sub_angles_from_square(const Square & square, float * sub_angles, const Point & roi_pos, const unsigned int & width, const unsigned int & height, const float * FOV_div2);

#endif
