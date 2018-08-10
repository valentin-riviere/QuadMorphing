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
#include "string_tools.h"
#include <fstream>
#include <iterator>
#include <cstdlib>

// Parametrization flag
#define ACQUISITION_CONTINUOUS

// Debug flag
//#define IMAGES	 Load images instead of camera use
//#define VERBOSE	// For fps view
//#define VIEWER_ON	// Active viewer
//#define SAVE_IMG	// Save images in results/
#define SAVE_RES	// Save results in results.txt
//#define BENCHMARK	// Print execution time of distincts parts

#ifdef IMAGES
	#define IMAGES_PATH "/home/odroid/Pictures/textures/800x600/V2/fps100/D3/*.png"
	#define FPS 100
#endif

using namespace cv;
using namespace Pylon;
using namespace std;

// Header for functions
void process(void);
float median(deque<float>);
void blur_resize(Mat &, Mat &);

/// Global variables
chrono::duration<double> Ts, T; // time between two frames (in s)
auto t_e = chrono::system_clock::now(), t_s = chrono::system_clock::now(), t_p = chrono::system_clock::now(), t_n = chrono::system_clock::now();
deque<float> tau_list;
vector<float> tau_vector;

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

// Time-to-collision estimation parameters
double sigma_gauss = 2;
uint16_t subsample = 5;
uint16_t n_median = 20;
float thresh_Et = 1e-3;

// OpenCV variables
uint16_t height = 600;
uint16_t width = 800;
uint16_t h_ss = round(height/subsample);
uint16_t w_ss = round(width/subsample);
Mat img_src, img_now, img_resized, img_prec, img_dx, img_dy, img_dt;
Mat edges;
uint64_t img_time = 0, img_time_prec = 0;

// Window Viewer
const char* window_name = "Viewer";

int main(int argc, char* argv[])
{
    int exitCode = 0; 	// The exit code
	uint32_t num_img = 0;	// Image index

#ifdef VIEWER_ON
	// Create a window and show image
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
	img_prec = Mat(h_ss,w_ss,CV_32F);
	img_now = Mat(h_ss,w_ss,CV_32F);
	img_resized = Mat(h_ss,w_ss,CV_8UC1);
	img_dx = Mat(h_ss,w_ss,CV_32F);
	img_dy = Mat(h_ss,w_ss,CV_32F);
	img_dt = Mat(h_ss,w_ss,CV_32F);
	edges = Mat(img_src.size(), img_src.type());

#ifndef IMAGES
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
				t_e = chrono::system_clock::now();

				camera.RetrieveResult(100,ptrGrabResult, TimeoutHandling_ThrowException);

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
					img_src.copyTo(img_prec);
					img_time_prec = img_time;
					img_src = Mat(img_src.size(), img_src.type(),(uint8_t*) ptrGrabResult->GetBuffer());
					img_time = ptrGrabResult->GetTimeStamp();

					// Blur image and resize it
					blur_resize(img_src,img_now);
					GaussianBlur(img_now,img_now,Size(0,0),sigma_gauss); // Blur image


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

					t_s = chrono::system_clock::now();
					Ts = t_s-t_e;

					cout << "Time between two loops : " << Ts.count() << "s" << " and time between two frames :" << (img_time-img_time_prec)*0.000000001 << "s" << endl;
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
		glob(path,list_path);		// Load list of images
		sort(list_path.begin(), list_path.end(), numeric_string_compare);	// Sort it

		for (size_t k = 0 ; k<list_path.size() ; k++)
		{
			t_e = chrono::system_clock::now();

			// Convert for openCV and get time
#ifndef BENCHMARK
			img_now.copyTo(img_prec);
			img_time_prec = 0;
			img_src = imread(list_path[k],IMREAD_GRAYSCALE);
			blur_resize(img_src,img_now);
			GaussianBlur(img_now,img_now,Size(0,0),sigma_gauss); // Blur image
#else
			t_p = chrono::system_clock::now();
			img_now.copyTo(img_prec);
			t_n = chrono::system_clock::now(); T = t_n-t_p;
			cout << "copy previous image : " << T.count() << endl;
			img_time_prec = 0;
			t_p = chrono::system_clock::now();
			img_src = imread(list_path[k],IMREAD_GRAYSCALE);
			t_n = chrono::system_clock::now(); T = t_n-t_p;
			cout << "imread : " << T.count() << endl;
			t_p = chrono::system_clock::now();
			blur_resize(img_src,img_now);
			t_n = chrono::system_clock::now(); T = t_n-t_p;
			cout << "Blur and resize: " << T.count() << endl;
			t_p = chrono::system_clock::now();
			GaussianBlur(img_now,img_now,Size(0,0),sigma_gauss); // Blur image
			t_n = chrono::system_clock::now(); T = t_n-t_p;
			cout << "Gaussian blur : " << T.count() << endl;
#endif

			img_time = 1000000000.0/FPS;	// in ns

			// Image processing (avoid first image)
			if (num_img != 0)
				process();

			t_s = chrono::system_clock::now();
			Ts = t_s-t_e;

			cout << "Time between two loops : " << Ts.count() << "s" << " and time between two frames :" << (img_time-img_time_prec)*0.000000001 << "s" << endl;

	#ifdef VIEWER_ON
			Mat img_show(height,width,CV_8UC1);
			img_now.convertTo(img_show,CV_8UC1);
			imshow(window_name,img_show); waitKey(0);
	#endif
			
			cout << "Image " << ++num_img << " above" << endl;
		}

	#ifdef SAVE_RES
		// Store in file results.txt
		ofstream output_file("./results.txt",ofstream::out);
		ostream_iterator<float> it(output_file,"\n");
		copy(tau_vector.begin(),tau_vector.end(),it);
	#endif

#endif

    return exitCode;
}

/**
 * @function process
 */
void process(void)
{
	float tau = 0, tau_fil; // Time to contact (in s)
	uint32_t it = 0;

#ifdef BENCHMARK
	t_p = chrono::system_clock::now();
#endif
	for (uint16_t i = 0 ; i < img_now.cols-1 ; i++)
	{
		for (uint16_t j = 0 ; j < img_now.rows-1 ; j++)
		{
			float Gx = (float) (1.0/4.0)*(img_prec.at<float>(j,i+1)-img_prec.at<float>(j,i)+img_prec.at<float>(j+1,i+1)-img_prec.at<float>(j+1,i)+img_now.at<float>(j,i+1)-img_now.at<float>(j,i)+img_now.at<float>(j+1,i+1)-img_now.at<float>(j+1,i));
			float Gy = (float) (1.0/4.0)*(img_prec.at<float>(j+1,i)-img_prec.at<float>(j,i)+img_prec.at<float>(j+1,i+1)-img_prec.at<float>(j,i+1)+img_now.at<float>(j+1,i)-img_now.at<float>(j,i)+img_now.at<float>(j+1,i+1)-img_now.at<float>(j,i+1));
			float Et = (float) (1.0/4.0)*(img_now.at<float>(j,i)-img_prec.at<float>(j,i)+img_now.at<float>(j+1,i)-img_prec.at<float>(j+1,i)+img_now.at<float>(j,i+1)-img_prec.at<float>(j,i+1)+img_now.at<float>(j+1,i+1)-img_prec.at<float>(j+1,i+1));

			float grad = (float) (i-(img_now.cols-1)/2.0)*Gx+(j-(img_now.rows-1)/2.0)*Gy;

			if (Et > thresh_Et)
			{
				tau -= grad/Et;
				it++;
			}
		}
	}

	// time to collision
	tau = (float) (tau / it) * (img_time-img_time_prec)*0.000000001;

	// Store in queue
	if (tau_list.size() > n_median)	// Keep only N_median values
		tau_list.pop_back();
	tau_list.push_front(tau);

#ifndef BENCHMARK
	// Perform median filtering
	tau_fil = median(tau_list);

	#ifdef SAVE_RES
	// Store in list
	tau_vector.push_back(tau_fil);
	#endif
#else
	t_n = chrono::system_clock::now(); T = t_n-t_p;
	cout << "tau computation : " << T.count() << endl;
	t_p = chrono::system_clock::now();

	// Perform median filtering
	tau_fil = median(tau_list);

	t_n = chrono::system_clock::now(); T = t_n-t_p;
	cout << "median filtering : " << T.count() << endl;
	t_p = chrono::system_clock::now();
#endif

	cout << "Tau : " << tau << "s" << endl;
	cout << "Tau filtered : " << tau_fil << "s" << endl;
}

float median(deque<float> list)
{
	float res;

	sort(list.begin(),list.end());

	uint16_t N = list.size();
	if (N % 2 != 0) // If odd number
		res = list[N/2];
	else
		res = (list[N/2 - 1] + list[N/2])/2.0;

	return res;
}

void blur_resize(Mat & src, Mat & dst)
{
	for (uint16_t i = 0 ; i < dst.cols ; i++)
	{
		for (uint16_t j = 0 ; j < dst.rows ; j++)
		{
			dst.at<float>(j,i) = (
src.at<uint8_t>(5*j,5*i)+src.at<uint8_t>(5*j+1,5*i)+src.at<uint8_t>(5*j+2,5*i)+src.at<uint8_t>(5*j+3,5*i)+src.at<uint8_t>(5*j+4,5*i)
+
src.at<uint8_t>(5*j,5*i+1)+src.at<uint8_t>(5*j+1,5*i+1)+src.at<uint8_t>(5*j+2,5*i+1)+src.at<uint8_t>(5*j+3,5*i+1)+src.at<uint8_t>(5*j+4,5*i+1)
+
src.at<uint8_t>(5*j,5*i+2)+src.at<uint8_t>(5*j+1,5*i+2)+src.at<uint8_t>(5*j+2,5*i+2)+src.at<uint8_t>(5*j+3,5*i+2)+src.at<uint8_t>(5*j+4,5*i+2)
+
src.at<uint8_t>(5*j,5*i+3)+src.at<uint8_t>(5*j+1,5*i+3)+src.at<uint8_t>(5*j+2,5*i+3)+src.at<uint8_t>(5*j+3,5*i+3)+src.at<uint8_t>(5*j+4,5*i+3)
+
src.at<uint8_t>(5*j,5*i+4)+src.at<uint8_t>(5*j+1,5*i+4)+src.at<uint8_t>(5*j+2,5*i+4)+src.at<uint8_t>(5*j+3,5*i+4)+src.at<uint8_t>(5*j+4,5*i+4)
)/25.0;
		}
	}
}
