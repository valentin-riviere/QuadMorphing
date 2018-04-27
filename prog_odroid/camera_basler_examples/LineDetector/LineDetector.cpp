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
#include <queue>
#include "Corner.h"
#include "custom_functions.h"
#include "opencv_functions.h"

// Image processing
#define UNDISTORT // Undistort image
#define BLUR // Blur images

// Detectors
//#define CANNY
//#define LSD
#ifndef LSD
//	#define FLD
#endif
//#define HARRIS
#if defined FLD || defined LSD
	#define RECT // Rectangle detection (need FLD or LSD)
#endif
#define SQUARE

// Parametrization flag
#define ACQUISITION_CONTINUOUS

// Usb flag for Frame Rate limitation but slow compilation
#define USB_CAMERA

// Benchmark
#define TIME_EXEC

// Debug flag		
#ifndef TIME_EXEC
	#define VIEWER_ON	// Active viewer
//	#define SAVE_IMG	// Save images in results/

	#ifdef FLD
//		#define DEBUG_FLD_PARAM	// Active to debug fld with sliders
//		#define DEBUG_FLD_LINE	// Active FLD debug line per line
	#endif
	#ifdef HARRIS
		//#define DEBUG_HARRIS// Active FLD debug point per point
	#endif
	#ifdef RECT
		#define DEBUG_RECT // Active to debug with rect
	#endif
	#ifdef SQUARE
//		 #define DEBUG_SQUARE // Active to debug square by square
	#endif

	#if defined DEBUG_FLD_LINE || defined DEBUG_FLD_PARAM || defined DEBUG_HARRIS || defined DEBUG_RECT || defined SQUARE
		#define VIEWER_ON
	#endif
#endif

// Useful Macros
#define PI 3.14159265

using namespace cv;
using namespace Pylon;
using namespace std;

// Headers
void CannyDetector(void);
void LSDDetector(void);
void FLDetector(void);
void HarrisDetector(void);
void RectangleDetector(void);
void SquaresDetector(void);

/// Global variables

// For timing benchmark
std::chrono::high_resolution_clock::time_point t_start, t_end;

// Name of windows
const char* window_src_name = "Image src";
const char* window_dst_name = "Image dst";

// Parameters for undistortion
string path_dist_coeffs("dist_coeffs_acircles.yml");

// Parameters for blur
const uint8_t size_blur = 3;

// Parameters for Canny Detection
const uint16_t width = 800, height = 600;
const double thresh_canny = 80;
const double ratio_canny = 3;
const int kernel_size_canny = 3;

// Parameters for LSD Detection
const int refine_lsd = LSD_REFINE_STD;	// LSD_REFINE_NONE, STD, ADV
const double scale_lsd = 0.8;
const double sigma_lsd = 0.6;
const double quant_lsd = 2.0;
const double ang_lsd = 10;
const double log_eps_lsd = 0;
const double density_lsd = 0.7;
const int n_bins_lsd = 1024;

// Parameters for FL Detection
#ifndef DEBUG_FLD_PARAM
	const int length_thresh_fl = 40; // 10
	const float dist_thresh_fl = 5; // 1.414213562f
	const double canny_th1_fl = 400.0; // 50
	const double canny_th2_fl = 700.0; // 50
	const int canny_aper_size_fl = 3;
	const bool do_merge_fl = false;
#else
	int length_thresh_fl = 40;
	int dist_thresh_fl = 5;
	int canny_th1_fl = 400;
	int canny_th2_fl = 700;
	int canny_aper_size_fl = 3;
	int do_merge_fl = false;

	#define DEBUG_PARAM &do_merge_fl
	const int minTrackbar = 0;
	const int maxTrackbar = 1;
#endif

// Parameters for Harris Corner Detection
const int blockSize_harris = 2;
const int kSize_harris = 3;
const double k_harris = 0.04;
const BorderTypes borderType_harris = BORDER_DEFAULT; 
const int thresh_harris = 180;

// Parameters for Rectangle Detection;
const float thresh_angle = 20.0*PI/180.0;
#ifndef DEBUG_RECT
	const float r_square_detect = 10*10;	// square radius (in px²) for detection
#else
	int r_square_detect = 10*10;
	
	const int minTrackbar_rect = 100;
	const int maxTrackbar_rect = 10000;
#endif

// Parameters for squares detection
#ifndef DEBUG_SQUARE
const int thresh_bin_square = 50;		// Threshold to digitize image
#else
int thresh_bin_square = 50;

const int minTrackbar_square = 0;
const int maxTrackbar_square = 255;
#endif
const float k_approx_square = 0.02; 	// Ratio of perimeter for approximation error
const float thresh_area_square = 1000.0;// Threshold for minimum area to detect
const float thresh_cos_square = 0.2;	// Threshold on cos condition

// Number of images to be grabbed if no continuous acquisition
const uint32_t c_countOfImagesToGrab = 1000;

// Camera Parameters
const uint8_t MaxNumBuffer = 5;
const uint16_t cam_width = 800, cam_height = 600;
const uint16_t cam_offsetX = 0, cam_offsetY = 0;
const bool binning_on = true;
const gcstring binning_mode("Average"); // Binning mode (Average/Sum)
const uint16_t fps_max = 300;	// FPS max if USB_CAMERA enable
const float expoAutoMax = 5000; // Max time for auto exposure (in us)

// OpenCV variables
Mat img_src, img_dst, img_tmp, img_to_print;
Mat edges;
uint64_t img_time;
vector<Vec4f> lines_std, lines_keep;	// Output of LSD Detector / FLDetector / RectDetector
priority_queue<Corner, vector<Corner>, less<vector<Corner>::value_type> > corners;	// Output of Harris Detector
vector<vector<Point> > squares;	// Output for Squares Detector

int main(int argc, char* argv[])
{
    int exitCode = 0; 	// The exit code
	int num_img = 0;	// Image index

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

	// OpenCV var init
	img_src = Mat(height,width,CV_8UC1);
	img_dst = Mat(height,width,CV_8UC1);
	img_tmp = Mat(height,width,CV_8UC1);
	img_to_print = Mat(height,width,CV_8UC3);
	edges = Mat(img_src.size(), img_src.type());
	Mat cameraMatrix, distCoeffs;

    // Before using any pylon methods, the pylon runtime must be initialized. 
    PylonInitialize();

    try
    {
#ifdef UNDISTORT
		// Load distortion coefficients and matrix calibration
		opencv_dist_matrix_from_file(cameraMatrix, distCoeffs, path_dist_coeffs);
#endif

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
#ifdef USB_CAMERA
		camera.AcquisitionFrameRate.SetValue(fps_max);
#endif

		// Start grabbing
		CGrabResultPtr ptrGrabResult;
#ifdef ACQUISITION_CONTINUOUS
		camera.StartGrabbing(GrabStrategy_LatestImageOnly);
#else
		camera.StartGrabbing(c_countOfImagesToGrab, GrabStrategy_LatestImageOnly);
#endif
		while (camera.IsGrabbing())
		{
#ifdef TIME_EXEC
			t_start = std::chrono::high_resolution_clock::now();
#endif
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
				img_src.copyTo(img_tmp); // Copy data to temp mat
				img_time = ptrGrabResult->GetTimeStamp();

#ifdef UNDISTORT
				undistort(img_tmp, img_src, cameraMatrix, distCoeffs);
#endif

#ifdef BLUR
				// Reduce noise with blur
				blur(img_src, img_dst, Size(size_blur,size_blur));
#endif

#ifdef CANNY
				// Canny detector and store results in global var edges
				CannyDetector();
#endif

#ifdef LSD
				// LSD Detector
				LSDDetector();
#endif

#ifdef FLD
	#ifdef DEBUG_FLD_PARAM // Create trackbar for debug parameters
				createTrackbar("Trackbar : ", window_dst_name, DEBUG_PARAM, maxTrackbar);
				setTrackbarMin("Trackbar : ", window_dst_name, minTrackbar);
	#endif
				// FLD Detector
				FLDetector();
#endif

#ifdef HARRIS
				// Detecting corners
				HarrisDetector();
#endif

#ifdef RECT
	#ifdef DEBUG_RECT
				createTrackbar("Trackbar rect : ", window_dst_name, &r_square_detect, maxTrackbar_rect);
				setTrackbarMin("Trackbar rect : ", window_dst_name, minTrackbar_rect);
	#endif
				// Rectangle Detector
				RectangleDetector();
#endif

#ifdef SQUARE
				// Squares Detector [Suzuki 85]
				SquaresDetector();
#endif

#ifdef VIEWER_ON
				imshow(window_src_name,img_src); waitKey(10);
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
			t_end = std::chrono::high_resolution_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);
			cout << "\t" << duration.count() << "ms" << " (" << 1000.0/duration.count() << "FPS)" << endl;
#endif
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
	blur(img_src, img_dst, Size(3,3));

	// Canny detector
	Canny(img_dst, img_dst, thresh_canny, thresh_canny*ratio_canny, kernel_size_canny);
}

/**
 * @function LSDDetector
 */
void LSDDetector()
{
	// Create and LSD detector with standard parameters for refinement
	Ptr<LineSegmentDetector> ls = createLineSegmentDetector(refine_lsd,scale_lsd,sigma_lsd,quant_lsd,ang_lsd,log_eps_lsd,density_lsd,n_bins_lsd);

	// Detect lines
	lines_std.clear();
	ls->detect(img_src,lines_std);

#if defined VIEWER_ON || defined SAVE_IMG
	// Draw lines
	img_to_print = Scalar(0);
	ls->drawSegments(img_to_print, lines_std);
#endif
}

/**
 * @function FLDetector
 */
void FLDetector()
{
	using namespace cv::ximgproc;

	// Create and LSD detector with standard parameters for refinement
	Ptr<FastLineDetector> fld = createFastLineDetector(length_thresh_fl,dist_thresh_fl,canny_th1_fl,canny_th2_fl,canny_aper_size_fl,do_merge_fl);

	// Detect lines
	lines_std.clear();
	fld->detect(img_src,lines_std);

#if defined VIEWER_ON || defined SAVE_IMG
	// Draw lines
	img_to_print = Scalar(0);

	#ifndef DEBUG_FLD_LINE 
		fld->drawSegments(img_to_print, lines_std);
	#else
		vector<Vec4f> lines_draw;
		for (vector<Vec4f>::iterator it = lines_std.begin() ; it != lines_std.end() ; ++it)
		{
			lines_draw.push_back(*it);
			fld->drawSegments(img_to_print, lines_draw);
			imshow(window_dst_name,img_to_print); waitKey(0);
		}
	#endif
#endif
}

/**
 * @function HarrisDetector
 */
void HarrisDetector()
{
	Mat dst(img_src.size(),CV_32FC1);

	// Detect corners
	cornerHarris(img_src, dst, blockSize_harris, kSize_harris, k_harris, borderType_harris );

	// Normalizing
  	normalize( dst, dst, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );

	// Threshold
	for( int j = 0; j < dst.rows ; j++ )
	{
		for( int i = 0; i < dst.cols; i++ )
		{
			if( (int) dst.at<float>(j,i) > thresh_harris )
			{
				Corner c(Point(i,j),dst.at<float>(j,i));
				corners.push(c);
			}
		}
	}

#ifdef VIEWER_ON
	// Convert scale abs for imshow
	convertScaleAbs( dst, img_to_print );

	cout << "Number of detected corners : " << corners.size() << endl;
	while (!corners.empty())
	{
		Corner c = corners.top();
		Point* p = c.getPoint();
		circle(img_to_print,*p,5,Scalar(0));
		corners.pop();
#ifdef DEBUG_HARRIS
		imshow(window_dst_name,img_to_print); waitKey(0);
#endif
	}
#endif
}

/**
 * @function DetectRectangle
 */
void RectangleDetector()
{
	float angle_offset = 0;	// In radian

	// Clear results
	lines_keep.clear();

	// Keep horizontal/vertical lines
	vector<Vec4f> lines_hor; 	// Horizontal lines results
	vector<Vec4f> lines_vert; 	// Vertical lines results
	for (unsigned int i = 0 ; i < lines_std.size() ; i++)
	{
		float angle_line = PI + atan2(lines_std[i][3]-lines_std[i][1],lines_std[i][2]-lines_std[i][0]);
		if (abs(angle_offset - fmod(angle_line,PI)) <= thresh_angle)
		{
			lines_hor.push_back(lines_std[i]);
		}
		else if (abs(PI/2.0 + angle_offset - fmod(angle_line,PI)) <= thresh_angle)
		{
			lines_vert.push_back(lines_std[i]);
		}
	}

	// Keep connected vertical/horizontal lines
	for (unsigned int i = 0 ; i < lines_hor.size() ; i++)
	{
		for (unsigned int j = 0 ; j < lines_vert.size() ; j++)
		{
			if ( ((lines_hor[i][0]-lines_vert[j][0])*(lines_hor[i][0]-lines_vert[j][0]) + (lines_hor[i][1]-lines_vert[j][1])*(lines_hor[i][1]-lines_vert[j][1]) <= r_square_detect) || ((lines_hor[i][0]-lines_vert[j][2])*(lines_hor[i][0]-lines_vert[j][2]) + (lines_hor[i][1]-lines_vert[j][3])*(lines_hor[i][1]-lines_vert[j][3]) <= r_square_detect) )
			{
				lines_keep.push_back(lines_hor[i]);
				lines_keep.push_back(lines_vert[j]);
			}
		}
	}

#ifdef VIEWER_ON
	cout << "Lines keeped for Rectangle Detector : " << endl;

	for (unsigned int i = 0 ; i < lines_hor.size() ; i++)
	{
		Point p1(lines_hor[i][0],lines_hor[i][1]), p2(lines_hor[i][2],lines_hor[i][3]);
		line(img_to_print,p1,p2,Scalar(255,0,255)); 	// Lines in purple
	}

	for (unsigned int i = 0 ; i < lines_vert.size() ; i++)
	{
		Point p1(lines_vert[i][0],lines_vert[i][1]), p2(lines_vert[i][2],lines_vert[i][3]);
		line(img_to_print,p1,p2,Scalar(255,0,255));	// Lines in purple
	}

	for (unsigned int i = 0 ; i < lines_keep.size() ; i++)
	{
		cout << "(" << lines_keep[i][0] << "," << lines_keep[i][1] << ")<->(" << lines_keep[i][2] << "," << lines_keep[i][3] << ")" << endl;
		Point p1(lines_keep[i][0],lines_keep[i][1]), p2(lines_keep[i][2],lines_keep[i][3]);
		line(img_to_print,p1,p2,Scalar(0,255,0));	// Lines in green
	}
#endif
}

/**
 * @function SquaresDetector
 */
void SquaresDetector()
{
	Mat gray(img_dst.size(), CV_8UC1);
	vector<vector<Point> > contours;

	// Clear detected squares
	squares.clear();

	// Threshold for binary image, could be remplaced by CANNY
	gray = img_dst >= thresh_bin_square;
	
 	// find contours and store them all as a list
    findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

    vector<Point> approx;

    // test each contour
    for( size_t i = 0; i < contours.size(); i++ )
    {
        // approximate contour with accuracy proportional
        // to the contour perimeter
        approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*k_approx_square, true);

        // square contours should have 4 vertices after approximation
        // relatively large area (to filter out noisy contours)
        // and be convex.
        // Note: absolute value of an area is used because
        // area may be positive or negative - in accordance with the
        // contour orientation
        if( approx.size() == 4 &&
            fabs(contourArea(Mat(approx))) > thresh_area_square &&
            isContourConvex(Mat(approx)) )
        {
            double maxCosine = 0;

            for( int j = 2; j < 5; j++ )
            {
                // find the maximum cosine of the angle between joint edges
                double cosine = fabs(cst_angle(approx[j%4], approx[j-2], approx[j-1]));
                maxCosine = MAX(maxCosine, cosine);
            }

            // if cosines of all angles are small
            // (all angles are ~90 degree) then write quandrange
            // vertices to resultant sequence
            if( maxCosine < thresh_cos_square)
			{
                squares.push_back(approx);
			}
        }
    }

#ifdef VIEWER_ON
	cvtColor(img_src,img_to_print,COLOR_GRAY2BGR);
	namedWindow("Digitized img", CV_WINDOW_AUTOSIZE ); imshow("Digitized img",gray);
	for( size_t i = 0; i < squares.size(); i++ )
    {
		for (unsigned j = 0 ; j < 4 ; j++)
		{
			Point p1(squares[i][j]), p2(squares[i][(j+1)%4]);
			line(img_to_print,p1,p2,Scalar(0,255,0));	// Lines in green
		}
    }
#ifdef DEBUG_SQUARE	
	createTrackbar("Trackbar rect : ", "Digitized img", &thresh_bin_square, maxTrackbar_square);
	setTrackbarMin("Trackbar rect : ", "Digitized img", minTrackbar_square);
	imshow(window_dst_name,img_to_print); waitKey(0);
#endif
#endif
}
