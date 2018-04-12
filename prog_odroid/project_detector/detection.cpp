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
#include "detection.h"

int detection(float * sub_angles, const Parameters p) // Subtented angles Left/Up/Right/Down,parameters)
{
    int16_t exitCode = 0; 	// The exit code
	uint32_t num_img = 0;	// Image index
	uint64_t img_time;
	chrono::high_resolution_clock::time_point t_start, t_prev = chrono::high_resolution_clock::now();
	CGrabResultPtr ptrGrabResult;		// Ptr to results
	bool init_square_detection = false; // To initialize square detection
	uint16_t no_detect = 0;
	int8_t state;
	// OpenCV var init
	Mat img_src = Mat(p.height,p.width,CV_8UC1), img_dst = Mat(p.height,p.width,CV_8UC1), img_tmp = Mat(p.height,p.width,CV_8UC1), img_to_print = Mat(p.height,p.width,CV_8UC3);
	vector<Square > squares;	// Output for Squares Detector
	Square sel_square; 			// Selected square

	// Before using any pylon methods, the pylon runtime must be initialized. 
    PylonInitialize();

    try
    {
		// Initialize and configure camera
		CInstantCamera camera(CTlFactory::GetInstance().CreateFirstDevice());
		camera_init(&camera,p.width,p.height);

		// Start grabbing
		camera.StartGrabbing(GrabStrategy_LatestImageOnly);
		while (camera.IsGrabbing())
		{
			t_start = std::chrono::high_resolution_clock::now();
			if (chrono::duration_cast<chrono::milliseconds>(t_start - t_prev) >= p.t_polling)
			{
				camera.RetrieveResult(p.grab_time_out,ptrGrabResult, TimeoutHandling_ThrowException);

				if (ptrGrabResult->GrabSucceeded())
				{
					// Reinitialize timer
					t_prev = t_start;

					// Convert for openCV and get time
					img_src = Mat(img_src.size(), img_src.type(), (uint8_t*) ptrGrabResult->GetBuffer());

					// Copy data
					img_tmp = img_src;
					
					// Get acquisition time
					img_time = ptrGrabResult->GetTimeStamp();

	#ifdef BLUR
					// Reduce noise with blur
					blur(img_tmp, img_dst, Size(p.size_blur,p.size_blur));
					img_tmp = img_dst;
	#endif

					// Squares Detector [Suzuki 85]
					SquaresDetector(img_tmp,squares,p.thresh_bin_square,p.k_approx_square,p.thresh_area_square,p.thresh_cos_square);

					// Select square
					state = select_square(squares, sel_square, p.thresh_diff2, p.thresh_ratio2);

					if (!init_square_detection)	// Initialize detection
					{
						usleep(p.time_init);
						
						cout << "Initialization...";
						if (!sel_square.empty())
						{
							init_square_detection = true;
							no_detect = 0;
							cout << "OK";
						}
						cout << endl;
					}
					else
					{
						if (sel_square.empty())	// No detection
						{
							cout << "No detected squares: " << ++no_detect << "/" << p.max_no_detect << endl;
							if (no_detect >= p.max_no_detect)	// If (no detection >= max_no_detect) => reinitialization
							{
								init_square_detection = false;
								cout << "Reinitialization..." << endl;
							}
						}
						else	// Subtented angles
						{
							no_detect = 0;
							sub_angles_from_square(sel_square,sub_angles,p.width,p.height,p.FOV_div2);
						}
					}

					// Subtented angles
					if (init_square_detection)
						cout << "Subtented angles : " << 180.0/CV_PI*sub_angles[0] << "\t" << 180.0/CV_PI*sub_angles[1] << "\t" << 180.0/CV_PI*sub_angles[2] << "\t" << 180.0/CV_PI*sub_angles[3] << endl;

	#ifdef DEBUG
					if(no_detect)
					{
						switch (state)
						{
							case -1:
								cout << "No detected squares" << endl;
								break;
							case 0:
								cout << "No pair squares" << endl;
								break;
							case 1:
								cout << "No centered pair squares" << endl;
								break;
							case 2:
								cout << "No similar ratio pair squares" << endl;
								break;
							default:
								break;
						}
					}
	#endif
					num_img++;
				}
				else
		        {
		            cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
		        }
			}
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
