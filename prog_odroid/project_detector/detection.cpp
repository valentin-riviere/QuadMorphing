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

int detection(const Stream_in * p_s_in, Stream_out * p_s_out, bool * p_sh_start, sem_t * sem)
{
	int16_t exitCode = 0; 	// The exit code
	uint32_t num_img = 0;	// Image index
	chrono::high_resolution_clock::time_point t_start, t_prev = chrono::high_resolution_clock::now(), t_prev_print = chrono::high_resolution_clock::now();
	CGrabResultPtr ptrGrabResult;		// Ptr to results
	int16_t state;
	uint32_t overrun = 0;	// Nbr of overruns

	// Before using any pylon methods, the pylon runtime must be initialized. 
    PylonInitialize();

	// Create camera
	CInstantCamera camera(CTlFactory::GetInstance().CreateFirstDevice());

	// Main loop
	while (loop_main)
	{
		// Outputs
		float sub_angles[4] = {0,0,0,0};
		uint64_t img_time;
		uint16_t no_detect = 0;
		uint16_t init_square_detection = 0; // To initialize square detection
		sem_wait(sem);	// Critical section and copy stream in into tmp var
			Stream_in s_in_cpy = *p_s_in;
			bool com_start = *p_sh_start;
		sem_post(sem);
	
		// Wait until communication start
		if (com_start == 0)
		{
			cout << "Detection : Wait Serial Connection." << endl;
			usleep(WAIT_SERIAL_TIME);
		}
		else
		{
			cout << "Detection : Start." << endl;

			// OpenCV var init
			Mat img_src = Mat(s_in_cpy.height,s_in_cpy.width,CV_8UC1), img_dst = Mat(s_in_cpy.height,s_in_cpy.width,CV_8UC1), img_tmp = Mat(s_in_cpy.height,s_in_cpy.width,CV_8UC1), img_to_print = Mat(s_in_cpy.height,s_in_cpy.width,CV_8UC3);
			vector<Square > squares;	// Output for Squares Detector
			Square sel_square; 			// Selected square

			try
			{
				// Initialize and configure camera
				camera_init(&camera,s_in_cpy.width,s_in_cpy.height, s_in_cpy.offset_x, s_in_cpy.offset_y, s_in_cpy.max_num_buffer, s_in_cpy.binning, s_in_cpy.expo_auto_max);

				// Start grabbing
				camera.StartGrabbing(GrabStrategy_LatestImageOnly);
				while (camera.IsGrabbing() && com_start == 1 && loop_main)
				{
					t_start = std::chrono::high_resolution_clock::now();
					if (chrono::duration_cast<chrono::milliseconds>(t_start - t_prev) >= (chrono::milliseconds) s_in_cpy.t_poll)
					{
						uint16_t duration = chrono::duration_cast<chrono::milliseconds>(t_start - t_prev).count();

						// Reinitialize timer
						t_prev = t_start;

#ifdef PRINT_DEBUG
						if (duration > s_in_cpy.t_poll)
						{
							overrun++;
							cout << "Detection : Overrun! (" << overrun << ")" << endl;
						}
#endif

#ifdef TIME_EXEC
						cout << "Time between two processing frames (ms) : " << duration << " / " << s_in_cpy.t_poll << endl;
#endif

#ifdef TIME_EXEC
t_start = std::chrono::high_resolution_clock::now();
#endif

						// Critical section and copy stream in into tmp var
						sem_wait(sem);
							s_in_cpy = *p_s_in;
							com_start = *p_sh_start;
						sem_post(sem);

						// Acquisition
						if (camera.RetrieveResult(s_in_cpy.t_poll,ptrGrabResult, TimeoutHandling_Return))
						{

	#ifdef TIME_EXEC
	cout << "Acquisition and copy shared vars (ms) : " << chrono::duration_cast<chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t_start).count() << endl;
	#endif

							if (ptrGrabResult->GrabSucceeded())
							{
								img_src = Mat(img_src.size(), img_src.type(), (uint8_t*) ptrGrabResult->GetBuffer());

	#ifdef TIME_EXEC
		cout << "Convertion (ms) : " << chrono::duration_cast<chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t_start).count() << endl;
		t_start = std::chrono::high_resolution_clock::now();
	#endif

								// Copy data
								img_tmp = img_src;
			
								// Get acquisition time
								img_time = ptrGrabResult->GetTimeStamp();

	#ifdef BLUR
								// Reduce noise with blur
								blur(img_tmp, img_dst, Size(s_in_cpy.size_blur,s_in_cpy.size_blur));
								img_tmp = img_dst;
	#endif

	#ifdef TIME_EXEC
		cout << "Blur (ms) : " << chrono::duration_cast<chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t_start).count() << endl;
		t_start = std::chrono::high_resolution_clock::now();
	#endif

								// Squares Detector [Suzuki 85]
								SquaresDetector(img_tmp,squares,s_in_cpy.bin_square,s_in_cpy.k_square,s_in_cpy.area_square,s_in_cpy.cos_square);

	#ifdef TIME_EXEC
		cout << "Square detection (ms) : " << chrono::duration_cast<chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t_start).count() << endl;
		t_start = std::chrono::high_resolution_clock::now();
	#endif

								// Select square
								state = select_square(squares, sel_square, s_in_cpy.thresh_diff2, s_in_cpy.thresh_ratio2);

	#ifdef TIME_EXEC
		cout << "Square selection (ms) : " << chrono::duration_cast<chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t_start).count() << endl;
		t_start = std::chrono::high_resolution_clock::now();
	#endif

								if (!init_square_detection)	// Initialize detection
								{	
	#ifdef DEBUG_DETECTION				
									cout << "Initialization...";
	#endif
									if (!sel_square.empty())
									{
										init_square_detection = 1;
										no_detect = 0;
										sub_angles_from_square(sel_square,sub_angles,s_in_cpy.width,s_in_cpy.height,s_in_cpy.fov);
										cout << "Detection OK";
									}
									cout << endl;
								}
								else
								{
									if (sel_square.empty())	// No detection
									{
										cout << "No detected squares: " << ++no_detect << "/" <<  s_in_cpy.max_no_detect << endl;
										if (no_detect >= s_in_cpy.max_no_detect)	// If (no detection >= max_no_detect) => reinitialization
										{
											init_square_detection = 0;
											cout << "Reinitialization..." << endl;
										}
									}
									else	// Subtented angles
									{
										no_detect = 0;
										sub_angles_from_square(sel_square,sub_angles,s_in_cpy.width,s_in_cpy.height,s_in_cpy.fov);
									}
								}

	#ifdef TIME_EXEC
		cout << "Sub angles computation (ms) : " << chrono::duration_cast<chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t_start).count() << endl;
		t_start = std::chrono::high_resolution_clock::now();
	#endif

	#ifdef PRINT_SUBANGLES
								// Print Subtented angles
								if (init_square_detection)
									cout << "Frame " << img_time << " : Subtented angles (in rad): " << sub_angles[0] << "\t" << sub_angles[1] << "\t" << sub_angles[2] << "\t" << sub_angles[3] << endl;
	#endif

	#ifdef DEBUG_DETECTION
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

	sem_wait(sem);	// Critical Section
								// Update Data Output Stream
								p_s_out->sub_angles[0]=sub_angles[0];
								p_s_out->sub_angles[1]=sub_angles[1];
								p_s_out->sub_angles[2]=sub_angles[2];
								p_s_out->sub_angles[3]=sub_angles[3];
								p_s_out->time_frame=img_time;
								p_s_out->no_detect=no_detect;
								p_s_out->init_detect=init_square_detection;
								com_start = *p_sh_start;
	sem_post(sem);

	#ifdef TIME_EXEC
		cout << "Copy to class (ms) : " << chrono::duration_cast<chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t_start).count() << endl;
		t_start = std::chrono::high_resolution_clock::now();
	#endif

								// Increment frame number
								num_img++;
							}
							else
							{
								cout << "Error on grab : " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
							}
						}
					}
#ifdef PRINT_DEBUG
					else if (chrono::duration_cast<chrono::milliseconds>(t_start - t_prev_print) >= (chrono::milliseconds) T_PRINT)
					{
						t_prev_print = t_start;
						cout << "Detection Task (" << s_in_cpy.t_poll << "ms)" << endl;
						cout << "\t\t" << "OverRuns : " << overrun << "/" << num_img << " ticks" << endl;
					}
#endif
				}	// End while
				camera.StopGrabbing();
				camera_close(&camera);
			}
			catch (const GenericException &e)
			{
				// Error handling.
				cerr << "An exception occurred." << endl
				<< e.GetDescription() << endl;
				exitCode = 1;

				camera.StopGrabbing();
				camera_close(&camera);
			}
		}
	} // End main loop

    // Releases all pylon resources. 
    PylonTerminate();  

    return exitCode;
}
