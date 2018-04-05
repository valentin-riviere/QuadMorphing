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
#include "custom_functions.h"
#include "opencv_functions.h"
#include "camera.h"

// Image processing
//#define UNDISTORT // Undistort image (check if it's working with ROI)
#define BLUR // Blur images

// Benchmark
//#define TIME_EXEC

// Debug flag
#ifndef TIME_EXEC
	#define VIEWER_ON	// Active viewer
//	#define SAVE_IMG	// Save images in results/
#endif

using namespace cv;
using namespace Pylon;
using namespace std;

/// Global variables

// Polling time (in ms)
const chrono::milliseconds t_polling(20);

// Grabbing
const uint16_t grab_time_out = 1000; // Time out in ms
const uint16_t width = 800, height = 600;
const uint16_t max_no_detect = 10;	// Maximum non detection before reinitialization

// ROI
const uint16_t roi_offsets[] = {50, 50}; // Offset on each sides of aperture to define ROI

// Name of windows
const char* window_src_name = "Image src";
const char* window_dst_name = "Image dst";

// Parameters for undistortion
string path_dist_coeffs("dist_coeffs_acircles.yml");

// Parameters for blur
const uint8_t size_blur = 3;

// Parameters for squares detection
const uint8_t thresh_bin_square = 100;		// Threshold to digitize image
const float k_approx_square = 0.02; 	// Ratio of perimeter for approximation error
const float thresh_area_square = 1000.0;// Threshold for minimum area to detect
const float thresh_cos_square = 0.1;	// Threshold on cos condition

int main(int argc, char* argv[])
{
    int16_t exitCode = 0; 	// The exit code
	uint64_t num_img = 0;	// Image index
	uint64_t img_time;
	chrono::high_resolution_clock::time_point t_start, t_prev = chrono::high_resolution_clock::now(), t_begin = chrono::high_resolution_clock::now();
	CGrabResultPtr ptrGrabResult;	// Ptr to results
	bool init_square_detection = false; // To initialize square detection
	uint16_t no_detect = 0;
	// OpenCV var init
	Mat img_src = Mat(height,width,CV_8UC1), img_dst = Mat(height,width,CV_8UC1), img_tmp = Mat(height,width,CV_8UC1), img_to_print = Mat(height,width,CV_8UC3);
	Mat cameraMatrix, distCoeffs;
	vector<Square > squares;	// Output for Squares Detector
	Square sel_square; 	// Selected square
	Rect ROI(0,0,width,height);	// ROI

#ifdef VIEWER_ON
	// Create windows
	namedWindow(window_src_name, CV_WINDOW_AUTOSIZE );
	namedWindow(window_dst_name, CV_WINDOW_AUTOSIZE );
#endif

#ifdef SAVE_IMG
	// Path name, format for recorded images and index
	if (int err = system("rm -rf results/*")) cout << "Results files deleted" << endl;
	String_t path_rec_img("results/rec");
	char str_buffer[100];
	vector<int> save_param;
	save_param.push_back(CV_IMWRITE_PNG_STRATEGY_DEFAULT);
#endif

	// Before using any pylon methods, the pylon runtime must be initialized. 
    PylonInitialize();

    try
    {
#ifdef UNDISTORT
		// Load distortion coefficients and matrix calibration
		opencv_dist_matrix_from_file(cameraMatrix, distCoeffs, path_dist_coeffs);
#endif
		// Initialize and configure camera
		CInstantCamera camera(CTlFactory::GetInstance().CreateFirstDevice());
		camera_init(&camera,width,height);

		// Start grabbing
		camera.StartGrabbing(GrabStrategy_LatestImageOnly);
		while (camera.IsGrabbing())
		{
			t_start = std::chrono::high_resolution_clock::now();
#ifdef TIME_EXEC
			if (chrono::duration_cast<chrono::milliseconds>(t_start - t_prev) >= t_polling)
			{
#endif
				camera.RetrieveResult(grab_time_out,ptrGrabResult, TimeoutHandling_ThrowException);

				if (ptrGrabResult->GrabSucceeded())
				{
					// Reinitialize timer
					t_prev = t_start;

					// Convert for openCV and get time
					img_src = Mat(img_src.size(), img_src.type(),(uint8_t*) ptrGrabResult->GetBuffer());
					img_tmp = img_src(ROI);		// select ROI
					img_dst.release(); img_dst = Mat(ROI.size(),CV_8UC1);	// Resize destination matrix
					img_time = ptrGrabResult->GetTimeStamp();

	#ifdef UNDISTORT		// Doesn't work with ROI!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
					undistort(img_tmp, img_dst, cameraMatrix, distCoeffs);
					img_dst.copyTo(img_tmp);
	#endif

	#ifdef BLUR
					// Reduce noise with blur
					blur(img_tmp, img_dst, Size(size_blur,size_blur));
	#endif

	#ifdef CANNY
					// Canny detector and store results in global var edges
					CannyDetector();
	#endif

					// Squares Detector [Suzuki 85]
					SquaresDetector(img_dst,squares,thresh_bin_square,k_approx_square,thresh_area_square,thresh_cos_square);

					// Select square
					if (!init_square_detection)	// Initialize detection with the more centered square
					{
						usleep(100000);
						cout << "Initialization...";
						if (!squares.empty())
						{
							select_center_square(squares,sel_square,width,height);
							init_square_detection = true;
							no_detect = 0;
							update_roi(ROI,sel_square,roi_offsets,width,height);	// Update ROI
							cout << "OK";
						}
						cout << endl;
					}
					else
					{
						if (squares.empty())	// No detection
						{
							cout << "No detected squares: " << ++no_detect << "/" << max_no_detect << endl;
							if (no_detect >= max_no_detect)	// If (no detection >= max_no_detect) => reinitialization
							{
								init_square_detection = false;
								cout << "Reinitialization..." << endl;
								ROI = Rect(0,0,width,height);	// Reinitialize ROI
							}
						}
						else	// Select square
						{
							no_detect = 0;
							select_center_square(squares,sel_square,ROI.size().width,ROI.size().height); ////////// A MODIFIER
							update_roi(ROI,sel_square,roi_offsets,width,height);	// Update ROI
						}
					}

				

	#ifdef VIEWER_ON
	//				if (init_square_detection)
	//					draw_squares(img_tmp,sel_square,img_to_print);
	//				else
						draw_squares(img_tmp,squares,img_to_print);
					imshow(window_src_name,img_src);
					imshow(window_dst_name,img_to_print); waitKey(10);
	#endif

	#ifdef SAVE_IMG
					// Save
					sprintf(str_buffer,"results/rec_src_%d_%llu.png",num_img,img_time);
					imwrite(str_buffer,img_src,save_param);
					sprintf(str_buffer,"results/rec_dst_%d_%llu.png",num_img,img_time);
					imwrite(str_buffer,img_to_print,save_param);
	#endif
				}
				else
		        {
		            cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
		        }

				num_img++;
#ifdef TIME_EXEC
				auto duration = chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - t_start);
				auto duration_polling = chrono::duration_cast<chrono::milliseconds>(t_prev-t_begin); t_begin = t_prev;
				cout << "\tPolling : " << duration_polling.count() << "ms\n\tProcessing : " << duration.count() << "ms" << " (" << 1000.0/duration.count() << "FPS)" << endl;
			}
#endif
		}	// End while

		camera_close(&camera);
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
