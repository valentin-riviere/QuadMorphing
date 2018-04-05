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

using namespace cv;
using namespace Pylon;
using namespace std;
using namespace Basler_UsbCameraParams;

// Header for HarrisCorner function
void calculate_corner(Mat*,int,Mat*);

// Number of images to be grabbed.
static const uint32_t c_countOfImagesToGrab = 100;

// Threshold for Harris Corner Detection
int harris_threshold = 200;

int main(int argc, char* argv[])
{
    // The exit code of the sample application.
    int exitCode = 0;

	// Path name for recorded images and index
	String_t path_rec_img("results/rec");
	char str_buffer[100];
	int num_img = 0;
	queue<CPylonImage> ptrGrabResultList;
	CPylonImage pylonImg;

    // Before using any pylon methods, the pylon runtime must be initialized. 
    PylonInitialize();

    try
    {
        // Create an instant camera object with the camera device found first.
        CBaslerUsbCamera camera( CTlFactory::GetInstance().CreateFirstDevice());
		camera.Open();

        // Print the model name of the camera.
        cout << "Using device " << camera.GetDeviceInfo().GetModelName() << endl;

		// Camera parameters
		//camera.MaxNumBuffer = 5;
		camera.Width = 800; camera.OffsetX = 400;
		camera.Height = 600; camera.OffsetY = 300;
		camera.ReverseY = true;

		// This smart pointer will receive the grab result data.
		const size_t img_size = (size_t) (camera.PayloadSize.GetValue());
		uint8_t * const pBuffer = new uint8_t[img_size];
		CBaslerUsbCamera::StreamGrabber_t StreamGrabber(camera.GetStreamGrabber(0));
		StreamGrabber.Open();
		StreamGrabber.MaxBufferSize.SetValue(img_size);
		StreamGrabber.MaxNumBuffer.SetValue(1);
		StreamGrabber.PrepareGrab();
		const StreamBufferHandle hBuffer = StreamGrabber.RegisterBuffer(pBuffer, img_size);
		StreamGrabber.QueueBuffer(hBuffer,NULL);
		GrabResult Result;

		// sets up single acquisition.
		//camera.AcquisitionFrameRate = 100.0;
		camera.AcquisitionMode.SetValue( AcquisitionMode_SingleFrame );
		camera.ExposureMode.SetValue(ExposureMode_Timed);
		camera.ExposureTime.SetValue(10);

        // when c_countOfImagesToGrab images have been retrieved.
		uint32_t i = 0;
        while (i++<c_countOfImagesToGrab)
        {
			camera.AcquisitionStart.Execute();
			
			if (StreamGrabber.GetWaitObject().Wait(3000)) // Image grabbed successfully (timeout in ms)?
            {
				StreamGrabber.RetrieveResult(Result);

                // Access the image data.
                uint8_t *pImageBuffer = (uint8_t *) Result.Buffer();

				// Save grabbed result to file in results/ folder
				//sprintf(buffer,"results/rec%d.raw",num_img++);
				//path_rec_img = String_t(buffer);
				//CImagePersistence::Save(ImageFileFormat_Raw,path_rec_img,ptrGrabResult);
				//pylonImg.CopyImage(ptrGrabResult);
				//ptrGrabResultList.push(pylonImg);

				// Matrix image for opencv
				Mat cMatImg(Size(Result.GetSizeX(),Result.GetSizeY()),CV_8UC1,result.GetBuffer());
				
				Mat corner;
				calculate_corner(&cMatImg, harris_threshold,&corner);


            }
            else
            {
                cout << "Error: " << Result.GetErrorCode() << " " << Result.GetErrorDescription() << endl;
            }
        }

		// Image saving
/*		while (!ptrGrabResultList.empty())
		{
			cout << "Image " << num_img+1 << "/" << c_countOfImagesToGrab << endl;
			pylonImg = ptrGrabResultList.front();
			ptrGrabResultList.pop();
			sprintf(str_buffer,"results/rec%d.png",num_img++);
			pylonImg.Save(ImageFileFormat_Png,str_buffer);
			sprintf(str_buffer,"results/rec%d.raw",num_img);
			pylonImg.Save(ImageFileFormat_Raw,str_buffer);
		}*/

		StreamGrabber.Close();
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
