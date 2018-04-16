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
#ifndef DETECTION_H
#define DETECTION_H

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/types.hpp"
#include <pylon/PylonIncludes.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <cmath>
#include <chrono>
#include <unistd.h>
#include "class_types.h"
#include "drawing.h"
#include "custom_functions.h"
#include "img_processing.h"
#include "camera.h"

#define BLUR	// Blur images

using namespace cv;
using namespace Pylon;
using namespace std;

/// Default parameters

typedef struct
{
	// Timing parameters
	chrono::milliseconds t_polling; // Polling time (in ms)
	uint32_t time_init;				// Duration between two initialization (in us)

	// Grabbing
	uint16_t grab_time_out; 		// Time out in ms
	uint16_t width;
	uint16_t height;
	uint16_t max_no_detect;			// Maximum non detection before reinitialization

	// Angles FOV for subtented angles processing (in rad)
	float FOV_div2[2];				// FOV/2 {width,height}

	// Parameters for select square
	uint16_t thresh_diff2;			// Diff^2 between 2 square centers (in px^2)
	float thresh_ratio2;			// Diff between ratio^2 of 2 squares (no unit)

	// Parameters for blur
	uint8_t size_blur;

	// Parameters for squares detection
	uint8_t thresh_bin_square;		// Threshold to digitize image
	float k_approx_square; 			// Ratio of perimeter for approximation error
	float thresh_area_square;		// Threshold for minimum area to detect
	float thresh_cos_square;		// Threshold on cos condition
}Parameters;

#define DEF_PARAMETERS {.t_polling=chrono::milliseconds(20),.time_init=0,.grab_time_out=1000,.width=800,.height=600,.max_no_detect=10,.FOV_div2={CV_PI/180.0*15.0,CV_PI/180.0*15.0},.thresh_diff2=20*20,.thresh_ratio2=0.2,.size_blur=3,.thresh_bin_square=100,.k_approx_square=0.02,.thresh_area_square=1000.0,.thresh_cos_square=0.1}

int detection(float * sub_angles, const Parameters p = DEF_PARAMETERS);

#endif
