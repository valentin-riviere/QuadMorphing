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

// Benchmark
#define TIME_EXEC

// Debug flag
//#define DEBUG
#ifndef TIME_EXEC
	#define VIEWER_ON	// Active viewer
//	#define SAVE_IMG	// Save images in results/
#endif

// Use ROI
//#define ROI

// Image processing
#ifndef ROI
	//#define UNDISTORT // Undistort image (check if it's working with ROI) => seems to not working
#endif
//#define CANNY	// Use Canny before detecting squares
#define BLUR	// Blur images

using namespace cv;
using namespace Pylon;
using namespace std;

#endif
