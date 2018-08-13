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
#ifndef TTC_COMPUTATION_H
#define TTC_COMPUTATION_H

#include "custom_functions.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

//#define DEBUG_TTC // Print ttc for each image

using namespace cv;
using namespace std;

// Blur and resize image (depends on dst size), use avg kernel = 5
void blur_resize(const Mat & src, Mat & dst);

// Compute ttc (Ts : time between two frames (in s), thresh_Et : threshold on brightness derivative value, n_median : order of the median filter for tau)
float compute_ttc(const Mat & img_prev, const Mat & img_now, const float Ts, const float thresh_Et, const uint16_t n_median);

#endif
