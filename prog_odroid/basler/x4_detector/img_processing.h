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
#ifndef IMG_PROCESSING_H
#define IMG_PROCESSING_H

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/types.hpp"

#include "class_types.h"
#include "custom_functions.h"

#include <iostream>
#include <math.h>
#include <string.h>

// Debug Flags
//#define DEBUG_RECT_DETECT	// Use to debug Square detection

using namespace cv;
using namespace std;

// Load dist coeffs and matrix coeffs from file
void opencv_dist_matrix_from_file(Mat & cameraMatrix, Mat & distCoeffs, const string path);

// Fast Line Detector [JH Lee 14]
void FLDetector(const Mat & src, vector<Vec4f> & lines, const int length_thresh_fl, const float dist_thresh_fl, const double canny_th1_fl, const double canny_th2_fl, const int canny_aper_size_fl, const bool do_merge_fl);

// Harris Corner Detector
void HarrisDetector(const Mat & src, priority_queue_corners & corners, const int thresh_harris, const int blockSize_harris, const int kSize_harris, const double k_harris, const BorderTypes borderType_harris);

// Custom Rectangle Detector (angle_offset = rectangle orientation, r_square_detect = detection radius for corners, thresh_angle = condition for angle)
void RectangleDetector(const vector<Vec4f> & lines_src, vector<Vec4f> & lines_dst, const float angle_offset, const uint16_t r_square_detect, const float thresh_angle);

// Detect squares by contour detection [S Suzuki 85]
void SquaresDetector(const Mat & src, vector<Square> & squares, const int thresh_bin_square, const float k_approx_square, const float thresh_area_square, const float thresh_cos_square);

#endif
