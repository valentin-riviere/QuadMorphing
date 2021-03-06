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
#include <semaphore.h>
#include "class_def.h"
#include "class_types.h"
#include "drawing.h"
#include "custom_functions.h"
#include "img_processing.h"
#include "camera.h"
#include "ttc_computation.h"

//#define DEBUG_DETECTION
//#define TIME_EXEC		// To print execution time for each part of the code
//#define PRINT_SUBANGLES	// Print subangles

#define BLUR	// Blur images

// For time-to-collision computation
#define H_SS 120		// Image size for subsampling
#define W_SS 160
#define SIGMA_GAUSS 2 	// Sigma for gaussian blur

#define WAIT_SERIAL_TIME 100000 // Waiting time between checks for serial initialization (in us)

using namespace cv;
using namespace Pylon;
using namespace std;

int detection(const Stream_in * p_s_in, Stream_out * p_s_out, bool * p_sh_start, sem_t * sem);

#endif
