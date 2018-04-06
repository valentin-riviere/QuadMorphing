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
#ifndef DRAWING_H
#define DRAWING_H

#include "class_types.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"

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

#endif
