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
#ifndef CAMERA_H
#define CAMERA_H

#include <pylon/PylonIncludes.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>

using namespace Pylon;
using namespace std;

// Camera Parameters
 // Binning mode (Average/Sum)
 // Max time for auto exposure (in us)

// Initialize and configure camera
void camera_init(CInstantCamera* cam, const unsigned int width = 800, const unsigned int height = 600, const unsigned int offset_x = 400, 
		const unsigned int offset_y = 300, const unsigned int MaxNumBuffer = 5,
		const bool binning_on = false, const float expoAutoMax = 5000, const gcstring binning_mode = "Average");
GenApi::CCommandPtr camera_init_trigger(CInstantCamera* cam, const unsigned int width = 800, const unsigned int height = 600, const unsigned int offset_x = 400, 			const unsigned int offset_y = 300, const unsigned int MaxNumBuffer = 5,
		const bool binning_on = false, const float expoAutoMax = 5000, const gcstring binning_mode = "Average");
void camera_close(CInstantCamera* cam);

#endif
