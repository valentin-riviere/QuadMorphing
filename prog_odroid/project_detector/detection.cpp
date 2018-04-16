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

extern bool loop_main;

int detection(const Stream_in * p_s_in, Stream_out * p_s_out, uint8_t * p_sh_start)
{
	int16_t exitCode = 0; 	// The exit code
	uint32_t num_img = 0;	// Image index
	chrono::high_resolution_clock::time_point t_start, t_prev = chrono::high_resolution_clock::now();
	CGrabResultPtr ptrGrabResult;		// Ptr to results
	int8_t state;

	// Before using any pylon methods, the pylon runtime must be initialized. 
    PylonInitialize();

	// Main loop
	while (loop_main)
	{
		// Outputs
		float sub_angles[4] = {0,0,0,0};
		uint64_t img_time;
		uint8_t no_detect = 0;
		uint8_t init_square_detection = 0; // To initialize square detection	
	
		// Wait until communication start
		if (*p_sh_start == 0)
		{
			cout << "Detection : Wait Serial Connection." << endl;
			usleep(10000);
		}
		else
		{
			cout << "Detection : Start." << endl;

			// OpenCV var init
			Mat img_src = Mat(p_s_in->height,p_s_in->width,CV_8UC1), img_dst = Mat(p_s_in->height,p_s_in->width,CV_8UC1), img_tmp = Mat(p_s_in->height,p_s_in->width,CV_8UC1), img_to_print = Mat(p_s_in->height,p_s_in->width,CV_8UC3);
			vector<Square > squares;	// Output for Squares Detector
			Square sel_square; 			// Selected square

			try
			{
				// Initialize and configure camera
				CInstantCamera camera(CTlFactory::GetInstance().CreateFirstDevice());
				camera_init(&camera,p_s_in->width,p_s_in->height, p_s_in->offset_x, p_s_in->offset_y, p_s_in->max_num_buffer, p_s_in->binning, p_s_in->expo_auto_max);

				// Start grabbing
				camera.StartGrabbing(GrabStrategy_LatestImageOnly);
				while (camera.IsGrabbing() && *p_sh_start == 1 && loop_main)
				{
					t_start = std::chrono::high_resolution_clock::now();
					if (chrono::duration_cast<chrono::milliseconds>(t_start - t_prev) >= (chrono::milliseconds) p_s_in->t_poll)
					{
						camera.RetrieveResult(p_s_in->grab_time_out,ptrGrabResult, TimeoutHandling_ThrowException);

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
							blur(img_tmp, img_dst, Size((uint16_t) p_s_in->size_blur,(uint16_t) p_s_in->size_blur));
							img_tmp = img_dst;
			#endif

							// Squares Detector [Suzuki 85]
							SquaresDetector(img_tmp,squares,(uint16_t) p_s_in->bin_square,p_s_in->k_square,p_s_in->area_square,p_s_in->cos_square);

							// Select square
							state = select_square(squares, sel_square, p_s_in->thresh_diff2, p_s_in->thresh_ratio2);

							if (!init_square_detection)	// Initialize detection
							{
								usleep(50);
						
								cout << "Initialization...";
								if (!sel_square.empty())
								{
									init_square_detection = 1;
									no_detect = 0;
									cout << "OK";
								}
								cout << endl;
							}
							else
							{
								if (sel_square.empty())	// No detection
								{
									cout << "No detected squares: " << ++no_detect << "/" << (uint16_t) p_s_in->max_no_detect << endl;
									if ( no_detect >= p_s_in->max_no_detect)	// If (no detection >= max_no_detect) => reinitialization
									{
										init_square_detection = 0;
										cout << "Reinitialization..." << endl;
									}
								}
								else	// Subtented angles
								{
									no_detect = 0;
									sub_angles_from_square(sel_square,sub_angles,p_s_in->width,p_s_in->height,p_s_in->fov);
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

							// Update Dat Output Stream
							p_s_out->sub_angles[0]=sub_angles[0];
							p_s_out->sub_angles[1]=sub_angles[1];
							p_s_out->sub_angles[2]=sub_angles[2];
							p_s_out->sub_angles[3]=sub_angles[3];
							p_s_out->time_frame=img_time;
							p_s_out->no_detect=no_detect;
							p_s_out->init_detect=init_square_detection;

							// Increment frame number
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
		}
	} // End main loop

    // Releases all pylon resources. 
    PylonTerminate();  

    return exitCode;
}
