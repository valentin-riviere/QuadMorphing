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
#include "camera.h"

void camera_init(CInstantCamera* cam, const unsigned int width, const unsigned int height)
{
	// Create an instant camera object with the camera device found first, get parameters and open camera
	GenApi::INodeMap& nodemap = cam->GetNodeMap();
	cam->Open();

	// Print the model name of the camera.
	cout << "Using device " << cam->GetDeviceInfo().GetModelName() << endl;

	// Camera parameters
	cam->MaxNumBuffer = MaxNumBuffer;
	GenApi::CIntegerPtr widthPtr = nodemap.GetNode("Width");
	GenApi::CIntegerPtr heightPtr = nodemap.GetNode("Height");
	GenApi::CIntegerPtr offsetXPtr = nodemap.GetNode("OffsetX");
	GenApi::CIntegerPtr offsetYPtr = nodemap.GetNode("OffsetY");
	GenApi::CBooleanPtr reverseXPtr = nodemap.GetNode("ReverseX");
	GenApi::CBooleanPtr reverseYPtr = nodemap.GetNode("ReverseY");
	GenApi::CEnumerationPtr binningModePtr = nodemap.GetNode("BinningHorizontalMode");
	GenApi::CIntegerPtr binningValuePtr = nodemap.GetNode("BinningHorizontal");
	GenApi::CFloatPtr resultingFPSPtr = nodemap.GetNode("ResultingFrameRate");
	GenApi::CFloatPtr autoExpoLowLim = nodemap.GetNode("AutoExposureTimeLowerLimit");
	GenApi::CFloatPtr autoExpoUpLim = nodemap.GetNode("AutoExposureTimeUpperLimit");
	GenApi::CEnumerationPtr expoAuto = nodemap.GetNode("ExposureAuto");
	GenApi::CEnumerationPtr acquisitionMode = nodemap.GetNode("AcquisitionMode");
	GenApi::CCommandPtr acquisitionStart = nodemap.GetNode("AcquisitionStart");
	GenApi::CEnumerationPtr triggerSelector = nodemap.GetNode("TriggerSelector");
	GenApi::CEnumerationPtr triggerMode = nodemap.GetNode("TriggerMode");
	widthPtr->SetValue(cam_width); heightPtr->SetValue(cam_height);
	offsetXPtr->SetValue(cam_offsetX); offsetYPtr->SetValue(cam_offsetY);
	reverseXPtr->SetValue(true); reverseYPtr->SetValue(true);
	if (binning_on)
	{
		binningValuePtr->SetValue(2);
		binningModePtr->FromString(binning_mode);
	}
	autoExpoLowLim->SetValue(autoExpoLowLim->GetMin());
	autoExpoUpLim->SetValue(expoAutoMax);
	expoAuto->FromString("Continuous");
	acquisitionMode->FromString("Continuous");
	triggerSelector->FromString("FrameStart");
	triggerMode->FromString("Off");
}

void camera_close(CInstantCamera* cam)
{
	cam->Close();
}
