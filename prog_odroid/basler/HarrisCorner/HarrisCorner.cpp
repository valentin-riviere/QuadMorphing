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
#include <pylon/usb/BaslerUsbCamera.h>
#include <stdlib.h>
#include <stdio.h>
#include <queue>
#include "Corner.h"

using namespace cv;
using namespace Pylon;
using namespace std;

// Header for HarrisCorner function
void calculate_corner(Mat*,int,Mat*);

// Parameters for Harris Corner Detection
const int harris_threshold = 250;
const int blockSize = 2;
const int apertureSize = 3;
const double k = 0.04;

// Number of images to be grabbed.
static const uint32_t c_countOfImagesToGrab = 1000;

// Camera Parameters
const uint8_t MaxNumBuffer = 5;
const uint16_t width = 800, height = 600;
const uint16_t offsetX = 400, offsetY = 300;

int main(int argc, char* argv[])
{
    // The exit code of the sample application.
    int exitCode = 0;

	// Declare CV var
	Mat cvImageDST(height,width,CV_32FC1); 

	// Path name, format for recorded images and index
	String_t path_rec_img("results/rec");
	char str_buffer[100];
	int num_img = 0;
	vector<int> save_param;
	save_param.push_back(CV_IMWRITE_PXM_BINARY);

	// List of detected corners
//	queue<queue<Point>> corners;

    // Before using any pylon methods, the pylon runtime must be initialized. 
    PylonInitialize();

    try
    {
        // Create an instant camera object with the camera device found first, get parameters and open camera
        CInstantCamera camera(CTlFactory::GetInstance().CreateFirstDevice());
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
		widthPtr->SetValue(width); heightPtr->SetValue(height);
		offsetXPtr->SetValue(offsetX); offsetYPtr->SetValue(offsetY);
		reverseYPtr->SetValue(true);

		// Smart pointer for grab result
		CGrabResultPtr ptrGrabResult;

		// Start grabbing
		camera.StartGrabbing(c_countOfImagesToGrab, GrabStrategy_LatestImageOnly);

		// Image saving
		while (camera.IsGrabbing())
		{
			camera.RetrieveResult(10000,ptrGrabResult, TimeoutHandling_ThrowException);
			
			if (ptrGrabResult->GrabSucceeded())
			{
				cout << "Image " << num_img+1 << "/" << c_countOfImagesToGrab << endl;
				// Convert for openCV
				Mat cvImageSRC(height,width,CV_8UC1,(uint8_t*) ptrGrabResult->GetBuffer());

				// Detecting corners
  				cornerHarris( cvImageSRC, cvImageDST, blockSize, apertureSize, k, BORDER_DEFAULT );

				// Normalizing
  				normalize( cvImageDST, cvImageDST, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
  				convertScaleAbs( cvImageDST, cvImageDST );

				// Store corners
				priority_queue<Corner, vector<Corner>, less<vector<Corner>::value_type> > corners;
				for( int j = 0; j < cvImageDST.rows ; j++ )
				{
					for( int i = 0; i < cvImageDST.cols; i++ )
					{
						if( (int) cvImageDST.at<float>(j,i) > harris_threshold )
						{
							Corner c(Point(i,j),cvImageDST.at<float>(j,i));
							corners.push(c);
							cout << c;
						}
					}
				}

				cout << endl;

				//corners.push(corn);
				// Save
				//sprintf(str_buffer,"results/rec%d.pgm",num_img);
				//imwrite(str_buffer,cvImageDST,save_param);
				//imwrite(str_buffer,cvImageSRC);
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

void calculate_corner(Mat *p_src, int thresh, Mat *p_dst)
{
  Mat dst, dst_norm;
  dst = Mat::zeros( p_src->size() , CV_32FC1 );

  /// Detector parameters
  int blockSize = 2;
  int apertureSize = 3;
  double k = 0.04;

  /// Detecting corners
  cornerHarris( *p_src, dst, blockSize, apertureSize, k, BORDER_DEFAULT );

  /// Normalizing
  normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
  convertScaleAbs( dst_norm, *p_dst );

  /// Drawing a circle around corners
  for( int j = 0; j < dst_norm.rows ; j++ )
     {
		for( int i = 0; i < dst_norm.cols; i++ )
          {
            if( (int) dst_norm.at<float>(j,i) > thresh )
              {
               circle( *p_dst, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
              }
          }
     }
}
