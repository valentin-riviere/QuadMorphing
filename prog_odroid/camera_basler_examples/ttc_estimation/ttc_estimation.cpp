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
#include <chrono>
#include <ctime>

// Parametrization flag
#define ACQUISITION_CONTINUOUS

// Debug flag
#define IMAGES	// Load images instead of camera use
//#define VERBOSE	// For fps view
#define VIEWER_ON	// Active viewer
//#define SAVE_IMG	// Save images in results/

#ifdef IMAGES
	#define IMAGES_PATH "/home/odroid/Pictures/textures/without_gap/800x600/V2/fps40/gap0.3x0.2_sqSize0.05_D3/*.png"
	#define FPS 40









#endif

using namespace cv;
using namespace Pylon;
using namespace std;

// Header for functions
void InitProcess(void);
void process(void);

/// Global variables
float tau = 0; // Time to contact (in s)
chrono::duration<double> Ts; // time between two frames (in s)
auto t_s = chrono::system_clock::now();

// Number of images to be grabbed if no continuous acquisition
#ifndef ACQUISITION_CONTINUOUS
static const uint32_t c_countOfImagesToGrab = 1000;
#endif

// Camera Parameters
const uint8_t MaxNumBuffer = 5;
const uint16_t cam_width = 800, cam_height = 600;
const uint16_t cam_offsetX = 400, cam_offsetY = 300;
const uint16_t fps_max = 100;
const float time_exposition = 9000; // in us
const float gain = 5; // in dB

// OpenCV variables
uint16_t height = 600;
uint16_t width = 800;
Mat img_src, img_prec, img_dx, img_dy, img_dt;
Mat edges;
uint64_t img_time = 0, img_time_prec = 0;
Mat x,y;	// For ttc computation

int main(int argc, char* argv[])
{
    int exitCode = 0; 	// The exit code
	uint32_t num_img = 0;	// Image index

#ifdef VIEWER_ON
	// Create a window and show image
	const char* window_name = "Viewer";
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
	img_prec = Mat(height,width,CV_8UC1);
	img_dx = Mat(height,width,CV_16S);
	img_dy = Mat(height,width,CV_16S);
	img_dt = Mat(height,width,CV_16S);
	edges = Mat(img_src.size(), img_src.type());

#ifndef IMAGES
		// Before using any pylon methods, the pylon runtime must be initialized. 
		PylonInitialize();
	
		// Initialization for vars during img processing
	//	InitProcess();

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
			GenApi::CEnumerationPtr gainAuto = nodemap.GetNode("GainAuto");
			GenApi::CFloatPtr gainPtr = nodemap.GetNode("Gain");
			GenApi::CFloatPtr autoExpoLowLim = nodemap.GetNode("AutoExposureTimeLowerLimit");
			GenApi::CFloatPtr autoExpoUpLim = nodemap.GetNode("AutoExposureTimeUpperLimit");
			GenApi::CEnumerationPtr expoAuto = nodemap.GetNode("ExposureAuto");
			GenApi::CFloatPtr expoTime = nodemap.GetNode("ExposureTime");
			GenApi::CEnumerationPtr acquisitionMode = nodemap.GetNode("AcquisitionMode");
			GenApi::CCommandPtr acquisitionStart = nodemap.GetNode("AcquisitionStart");
			GenApi::CEnumerationPtr triggerSelector = nodemap.GetNode("TriggerSelector");
			GenApi::CEnumerationPtr triggerMode = nodemap.GetNode("TriggerMode");
			widthPtr->SetValue(cam_width); heightPtr->SetValue(cam_height);
			offsetXPtr->SetValue(cam_offsetX); offsetYPtr->SetValue(cam_offsetY);
			reverseYPtr->SetValue(true);
			gainAuto->FromString("Off");
			gainPtr->SetValue(gain);
			autoExpoLowLim->SetValue(autoExpoLowLim->GetMin());
			autoExpoUpLim->SetValue((float) 1000000.0/fps_max);
			expoTime->SetValue(9000.0);
			expoAuto->FromString("Off");
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
	//			auto t_e = chrono::system_clock::now();
	//			Ts = t_e-t_s;
	//			t_s = t_e;
			
				if (ptrGrabResult->GrabSucceeded())
				{
	#ifdef VERBOSE
	#ifdef ACQUISITION_CONTINUOUS
					cout << "Image " << num_img+1 << " (" << resultingFPSPtr->GetValue() << "FPS)" << endl;
	#else
					cout << "Image " << num_img+1 << "/" << c_countOfImagesToGrab << " (" << resultingFPSPtr->GetValue() << "FPS)" << endl;
	#endif
	#endif

					// Convert for openCV and get time
					img_prec = img_src;
					img_time_prec = img_time;
					img_src = Mat(img_src.size(), img_src.type(),(uint8_t*) ptrGrabResult->GetBuffer());
					img_time = ptrGrabResult->GetTimeStamp();

					// Image processing (avoid first image)
					if (num_img != 0)
						process();

	#ifdef VIEWER_ON
					Mat img_show(height,width,CV_8UC1);
					img_src.convertTo(img_show,CV_8UC1);
					imshow(window_name,img_show); waitKey(10);
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

#else	// With loaded images / without camera

		cout << "Using images in folder : " << IMAGES_PATH << endl;

		String path(IMAGES_PATH);
		vector<String> list_path;
		glob(path,list_path);
		sort(list_path.begin(), list_path.end(), numeric_string_compare);
		for (size_t k = 0 ; k<list_path.size() ; k++)
		{
			cout << list_path[k] << endl;

			// Convert for openCV and get time
			img_prec = img_src;
			img_time_prec = 0;
			img_src = imread(list_path[k],IMREAD_GRAYSCALE);
			img_time = 1000000000.0/FPS;	// in ns

			// Image processing (avoid first image)
			if (num_img != 0)
				process();

	#ifdef VIEWER_ON
			Mat img_show(height,width,CV_8UC1);
			img_src.convertTo(img_show,CV_8UC1);
			imshow(window_name,img_show); waitKey(0);
	#endif
			
			num_img++;
		}

#endif

    return exitCode;
}

/**
 * @function InitProcess
 */
void InitProcess(void)
{
	x = Mat(width,height,CV_16SC1);
	y = Mat(width,height,CV_16SC1);

	for (uint16_t i = 0 ; i < img_src.cols ; i++)
	{
		for (uint16_t j = 0 ; j < img_src.rows ; j++)
		{
			x.at<uint16_t>(j,i) = (-width/2 + j);
			y.at<uint16_t>(j,i) = (-height/2 + i);
		}
	}
}

/**
 * @function process
 */
void process(void)
{
	// dx, dy and dt
	spatialGradient(img_src,img_dx,img_dy);
	subtract(img_src,img_prec,img_dt,noArray(),CV_16SC1);

	// num and den for tau
	float num = 0, den = 0;
	for (uint16_t i = 0 ; i < img_src.cols ; i++)
	{
//cout << i+1 << "/" << img_src.cols << endl;
		for (uint16_t j = 0 ; j < img_src.rows ; j++)
		{
			int32_t grad = (i-(img_src.cols-1)/2.0)*img_dx.at<int16_t>(j,i)+(j-(img_src.rows-1)/2.0)*img_dy.at<int16_t>(j,i);
//cout << grad << "    " << img_dx.at<int16_t>(j,i) << "    " << img_dy.at<int16_t>(j,i) << endl;
//if (i > 0 && i < img_src.cols-1 && j > 0 && j < img_src.rows-1)
//{
//cout << i
//<< "    " << j 
//<< "    " << grad 
//<< "    " << img_dx.at<int16_t>(j,i) << "    " << img_dy.at<int16_t>(j,i)
//<< "    " << (uint16_t) img_src.at<uint8_t>(j-1,i-1)
//<< "    " << (uint16_t) img_src.at<uint8_t>(j-1,i) 
//<< "    " << (uint16_t) img_src.at<uint8_t>(j-1,i+1) 
//<< "    " << (uint16_t) img_src.at<uint8_t>(j,i-1) 
//<< "    " << (uint16_t) img_src.at<uint8_t>(j,i) 
//<< "    " << (uint16_t) img_src.at<uint8_t>(j,i+1) 
//<< "    " << (uint16_t) img_src.at<uint8_t>(j+1,i-1) 
//<< "    " << (uint16_t) img_src.at<uint8_t>(j+1,i) 
//<< "    " << (uint16_t) img_src.at<uint8_t>(j+1,i+1) 
//<< endl;
//}

			num -= grad*grad;
			den += grad*img_dt.at<int16_t>(j,i);
//cout << "dt : " << img_dt.at<int16_t>(j,i) << endl;
//cout << "grad : " << grad << endl;
//cout << "num : " << num << endl;
//cout << "den : " << den << endl;
		}
	}

	// time to collision
	float c =  den/(num*(img_time-img_time_prec)*0.000000001);
	if (c > 0.01)
		tau = 1.0/c;
	else	
		tau = 0;

//cout << "c : " << c << endl;
//cout << "num : " << num << endl;
//cout << "den : " << den << endl;
//cout << "(img_time-img_time_prec) : " << (img_time-img_time_prec) << endl;
//waitKey(0);

auto t_e = chrono::system_clock::now();
Ts = t_e-t_s;
t_s = t_e;

	cout << "Time between images : " << Ts.count() << "s" << " and " << img_time-img_time_prec << endl;
	cout << "Tau : " << tau << "s" << endl;
}
