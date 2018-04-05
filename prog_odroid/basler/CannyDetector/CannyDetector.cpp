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
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <pylon/PylonIncludes.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>
#include <stdlib.h>
#include <stdio.h>
#include <queue>

// Parametrization flag
#define ACQUISITION_CONTINUOUS

// Usb flag for Frame Rate limitation but slow compilation
//#define USB_CAMERA

// Debug flag
#define VIEWER_ON	// Active viewer
//#define SAVE_IMG	// Save images in results/

using namespace cv;
using namespace Pylon;
using namespace std;

// Header for Canny Threshold function
void CannyDetector(void);

/// Global variables

// Parameters for Canny Detection
const uint16_t width = 640, height = 480;
const double thresh = 50;
const double ratio = 3;
const int kernel_size = 3;

// Number of images to be grabbed if no continuous acquisition
#ifndef ACQUISITION_CONTINUOUS
static const uint32_t c_countOfImagesToGrab = 1000;
#endif

// Camera Parameters
const uint8_t MaxNumBuffer = 5;
const uint16_t cam_width = 640, cam_height = 480;
const uint16_t cam_offsetX = 320, cam_offsetY = 240;
const uint16_t fps_max = 300;	// FPS max if USB_CAMERA enable, determine exposure max

// OpenCV variables
Mat img_src;
Mat edges;
uint64_t img_time;

int main(int argc, char* argv[])
{
    int exitCode = 0; 	// The exit code
	int num_img = 0;	// Image index

#ifdef VIEWER_ON
	// Create a window and show image
	const char* window_name = "Canny Edge Detector";
	namedWindow(window_name, CV_WINDOW_AUTOSIZE );
#endif

#ifdef SAVE_IMG
	// Path name, format for recorded images and index
	if (int err = system("rm -rf results/*")) cout << "Results files deleted" << endl;
	String_t path_rec_img("results/rec");
	char str_buffer[100];
	vector<int> save_param;
	save_param.push_back(CV_IMWRITE_PNG_STRATEGY_DEFAULT);
#endif

	// Var init
	img_src = Mat(height,width,CV_8UC1);
	edges = Mat(img_src.size(), img_src.type());

    // Before using any pylon methods, the pylon runtime must be initialized. 
    PylonInitialize();

    try
    {
        // Create an instant camera object with the camera device found first, get parameters and open camera
#ifdef USB_CAMERA
		CBaslerUsbInstantCamera camera(CTlFactory::GetInstance().CreateFirstDevice());
#else
        CInstantCamera camera(CTlFactory::GetInstance().CreateFirstDevice());
#endif
		GenApi::INodeMap& nodemap = camera.GetNodeMap();
		camera.Open();

        // Print the model name of the camera.
        cout << "Using device " << camera.GetDeviceInfo().GetModelName() << endl;

		// Camera parameters
		camera.MaxNumBuffer = MaxNumBuffer;
		GenApi::CIntegerPtr widthPtr = nodemap.GetNode("Width");
		GenApi::CIntegerPtr heightPtr = nodemap.GetNode("Height");
		GenApi::CIntegerPtr offsetXPtr = nodemap.GetNode("OffsetX");
		GenApi::CIntegerPtr offsetYPtr = nodemap.GetNode("OffsetY");
		GenApi::CBooleanPtr reverseYPtr = nodemap.GetNode("ReverseY");
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
		reverseYPtr->SetValue(true);
		autoExpoLowLim->SetValue(autoExpoLowLim->GetMin());
		autoExpoUpLim->SetValue((float) 1000000.0/fps_max);
		expoAuto->FromString("Continuous");
		acquisitionMode->FromString("Continuous");
		triggerSelector->FromString("FrameStart");
		triggerMode->FromString("Off");
#ifdef USB_CAMERA
		camera.AcquisitionFrameRate.SetValue(fps_max);
#endif

		// Smart pointer for grab result
		CGrabResultPtr ptrGrabResult;

		// Start grabbing
#ifdef ACQUISITION_CONTINUOUS
		camera.StartGrabbing(GrabStrategy_LatestImageOnly);
#else
		camera.StartGrabbing(c_countOfImagesToGrab, GrabStrategy_LatestImageOnly);
#endif

		// Image saving
		while (camera.IsGrabbing())
		{
			camera.RetrieveResult(100,ptrGrabResult, TimeoutHandling_ThrowException);
			
			if (ptrGrabResult->GrabSucceeded())
			{
#ifdef ACQUISITION_CONTINUOUS
				cout << "Image " << num_img+1 << " (" << resultingFPSPtr->GetValue() << "FPS)" << endl;
#else
				cout << "Image " << num_img+1 << "/" << c_countOfImagesToGrab << " (" << resultingFPSPtr->GetValue() << "FPS)" << endl;
#endif

				// Convert for openCV and get time
				img_src = Mat(img_src.size(), img_src.type(),(uint8_t*) ptrGrabResult->GetBuffer());
				img_time = ptrGrabResult->GetTimeStamp();

				// Canny detector and store results in global var edges
				CannyDetector();

#ifdef VIEWER_ON
				imshow(window_name,edges); waitKey(10);
#endif

#ifdef SAVE_IMG
				// Save
				sprintf(str_buffer,"results/rec%d_%llu.png",num_img,img_time);
				imwrite(str_buffer,edges,save_param);
#endif
			}
			else
            {
                cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
            }

			num_img++;
		}

		camera.Close();
    }
    catch (const GenericException &e)
    {
        // Error handling.
        cerr << "An exception occurred." << endl
        << e.GetDescription() << endl;
        exitCode = 1;
    }

    // Comment the following two lines to disable waiting on exit.
    cerr << endl << "Press Enter to exit." << endl;
    while( cin.get() != '\n');
	
    // Releases all pylon resources. 
    PylonTerminate();  

    return exitCode;
}

/**
 * @function CannyDetector
 */
void CannyDetector()
{
	// Reduce noise with a kernel 3x3
	blur(img_src, edges, Size(3,3));

	// Canny detector
	Canny(edges, edges, thresh, thresh*ratio, kernel_size);
}
