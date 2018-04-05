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
// Grab.cpp
/*
   	Grab and save images
*/

#include <pylon/PylonIncludes.h>
#include <queue>
#include <iostream>
#include <stdlib.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/types.hpp"

using namespace Pylon;
using namespace std;
using namespace cv;

void help()
{
	cout << endl << "HELP :\n./Grab [-k:debug mode frame by frame] [saving_prefix=/results/rec] [nbr_images=1] [time_ms_fps=10]\n" << endl;
}

// Camera Parameters
const uint8_t MaxNumBuffer = 5;
const uint16_t cam_width = 800, cam_height = 600;
const uint16_t cam_offsetX = 0, cam_offsetY = 0;
const bool binning_on = true;
const gcstring binning_mode("Average"); // Binning mode (Average/Sum)
const uint16_t fps_max = 300;	// FPS max if USB_CAMERA enable
const float expoAutoMax = 5000; // Max time for auto exposure (in us)

int main(int argc, char* argv[])
{
	help();

    // The exit code of the sample application.
    int exitCode = 0;

	// Arguments
	String path_rec_img;
	uint32_t c_countOfImagesToGrab;
	uint16_t t_ms_fps;
	bool debug_on;
	cv::CommandLineParser parser(argc, argv,
        "{@saving_prefix|results/rec|}"
		"{@nbr_images|1|}"
		"{@time_ms_fps|10|}"
		"{k||}");
	
	path_rec_img = parser.get<String>(0);
	c_countOfImagesToGrab = parser.get<int>(1);
	t_ms_fps = parser.get<int>(2);
	if (parser.has("k"))
		debug_on = true;
	else
		debug_on = false;

	// Print configuration
	cout << "Save " << c_countOfImagesToGrab << " image(s) with prefix : " << path_rec_img << " (" << 1.0/(0.001*t_ms_fps) << "FPS), debug=" << debug_on << "\n" << endl;

	// Pylon var
	char str_buffer[100];
	int num_img = 0;
	queue<CPylonImage> ptrGrabResultList;
	CPylonImage pylonImg;

    // Before using any pylon methods, the pylon runtime must be initialized. 
    PylonInitialize();

    try
    {
        // Create an instant camera object with the camera device found first.
        CInstantCamera camera( CTlFactory::GetInstance().CreateFirstDevice());

		// Open camera
        camera.Open();

        // Print the model name of the camera.
        cout << "Using device " << camera.GetDeviceInfo().GetModelName() << endl;


		GenApi::INodeMap& nodemap = camera.GetNodeMap();
		// Camera parameters
		camera.MaxNumBuffer = MaxNumBuffer;
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


        // This smart pointer will receive the grab result data.
        CGrabResultPtr ptrGrabResult;

		// Interrupt video with keyboard and record with next images
		cout << "Begin video. Push any key to record images..." << endl;
		bool interruption_key = false;
		Mat opencv_img(cam_height,cam_width,CV_8UC1);
		namedWindow("Img", CV_WINDOW_AUTOSIZE );
		camera.StartGrabbing(GrabStrategy_LatestImageOnly);
        while ( camera.IsGrabbing() && !interruption_key )
        {
            // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
            camera.RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);

            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded())
            {
                opencv_img = Mat(opencv_img.size(), opencv_img.type(),(uint8_t*) ptrGrabResult->GetBuffer());

				imshow("Img",opencv_img);
				if ( waitKey(10) != -1 )
					interruption_key = true;
            }
            else
            {
                cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
            }
        }

		// Record images
		cout << "Begin recording..." << endl;
		unsigned int n_img = 0;
		while ( camera.IsGrabbing() && (n_img < c_countOfImagesToGrab) )
        {
            // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
            camera.RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);

            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded())
            {
                // Access the image data.
                const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();
				
				if (!debug_on)	// Debug off
				{
					pylonImg.CopyImage(ptrGrabResult);
					ptrGrabResultList.push(pylonImg);
					cout << ++n_img << "/" << c_countOfImagesToGrab << endl;
					waitKey(t_ms_fps);
				}
				else			// Debug on
				{
					imshow("Img",Mat(opencv_img.size(), opencv_img.type(),(uint8_t*) ptrGrabResult->GetBuffer()));
					if ( waitKey(10) != -1 )	// If key pressed
					{
						pylonImg.CopyImage(ptrGrabResult);
						ptrGrabResultList.push(pylonImg);
						cout << ++n_img << "/" << c_countOfImagesToGrab << endl;
					}
				}
            }
            else
            {
                cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
            }
        }

		// Destroy openCV windows
		destroyAllWindows();

		// Close camera
		camera.Close();

		// Image saving
		cout << "Saving..." << endl;
		while (!ptrGrabResultList.empty())
		{
			cout << "Image " << num_img+1 << "/" << c_countOfImagesToGrab << endl;
			pylonImg = ptrGrabResultList.front();
			ptrGrabResultList.pop();
			sprintf(str_buffer,"%s%d.png",path_rec_img.c_str(),num_img);
			pylonImg.Save(ImageFileFormat_Png,str_buffer);
			sprintf(str_buffer,"%s%d.raw",path_rec_img.c_str(),num_img);
			pylonImg.Save(ImageFileFormat_Raw,str_buffer);
			num_img++;
		}
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
