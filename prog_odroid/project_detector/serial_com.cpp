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
#include "serial_com.h"

extern bool loop_main;

ostream& operator<<(ostream &f, Stream_in const * p_s_in)
{
	f << "Stream_in : " << endl << p_s_in->t_poll << endl << (uint16_t) p_s_in->max_num_buffer << endl << p_s_in->width;
	return f;
}

int serial_com(Stream_in * p_s_in, const Stream_out * p_s_out, uint8_t * p_sh_start)
{
	bool socket_on = false;	// Connection between odroid and RCB 2 is off
	bool loop_on = true;	// Variable for program loop
	int bytes_written = 0, bytes_read = 0, gumstix_fail = 0;
	uint8_t read_buffer[255], write_buffer[255] ={0};
	const chrono::milliseconds Duration_com(POLLING_TIME_MS); // Polling time for communication
	chrono::high_resolution_clock::time_point cTime = chrono::high_resolution_clock::now(), pTime = chrono::high_resolution_clock::now();

	// Open and configure Serial Port
	if (open_serial_port("/dev/ttySAC0"))
	{
		cout << "Error with open and configure serial port: " << strerror(errno) << endl;
		return -1;
	}
	
	// Program loop
	while (loop_on && loop_main)
	{
		cTime = chrono::high_resolution_clock::now();
		if(chrono::duration_cast<chrono::milliseconds>(cTime - pTime) >= Duration_com)
		{
			pTime = cTime;
			if (!socket_on)	// If socket is not initialized
			{
				*p_sh_start = 0;		// Detection is off
				if (start_asked()) 	// If start received and connection is off
				{
					// Send start stream on Serial Port
					cout << "Received START stream..." << endl;
					send_start_serial();
					socket_on = true;
					gumstix_fail = 0;
					cout << "Connection established!" << endl;
				}
				else	// Wait until next pool
				{
					cout << "Wait START stream..." << endl;
					usleep(WAIT_CONNECTION_TIME);
				}
			}
			else	// Socket is initialized
			{
				// Write on Serial Port
				bytes_written = header_write_serial(write_buffer,NB_BYTE_TO_WRITE,HEADER);
				if (bytes_written < 0)	// If troubles, exit program
				{
					cout << "UART TX error: " << strerror(errno) << endl;
					loop_on = false;
				}

				// Read on Serial Port
				bytes_read = header_read_serial(read_buffer,NB_BYTE_TO_READ,HEADER);
				if (bytes_read < 0)		// If troubles, exit program
				{
					cout << "UART RX error: " << strerror(errno) << endl;
					loop_on = false;
				}
				else if (bytes_read != NB_BYTE_TO_READ)	// If nothing to read => increment gumstix fails
				{
					if (++gumstix_fail >= NB_GUMSTIX_FAILED)
						socket_on = false;
					cout << "No stream received " << gumstix_fail << " / " << NB_GUMSTIX_FAILED << endl;
					cout << "\t" << bytes_read << " bytes read! (" << NB_BYTE_TO_READ << " bytes expected)" << endl;
				}
				else	// read_buffer contains gumstix data
				{
					gumstix_fail = 0;	// Reset gumstix fails when receive something
				 	*p_sh_start = 1;		// Communication is active => detection is on
					cout << bytes_read << " bytes read!" << endl;
					convert_stream_in(read_buffer,p_s_in);	// Convert bytes to struct stream_in data format
				}
			}
		}
	}

	close_serial_port();

	return -1;
}

void convert_stream_in(const uint8_t * read_buffer, Stream_in * p_s_in)
{
	uint8_2_uint16(read_buffer,&p_s_in->t_poll,1);
	p_s_in->max_num_buffer = *(read_buffer+2);
	uint8_2_uint16(read_buffer+3,&p_s_in->width,1);
	uint8_2_uint16(read_buffer+5,&p_s_in->height,1);
	uint8_2_uint16(read_buffer+7,&p_s_in->offset_x,1);
	uint8_2_uint16(read_buffer+9,&p_s_in->offset_y,1);
	p_s_in->binning = *(read_buffer+11);
	uint8_2_uint16(read_buffer+12,&p_s_in->expo_auto_max,1);
	uint8_2_uint16(read_buffer+14,&p_s_in->grab_time_out,1);
	uint8_2_float(read_buffer+16,p_s_in->fov,2);
	p_s_in->max_no_detect = *(read_buffer+24);
	p_s_in->size_blur = *(read_buffer+25);
	p_s_in->bin_square = *(read_buffer+26);
	uint8_2_float(read_buffer+27,&p_s_in->k_square,1);
	uint8_2_uint16(read_buffer+31,&p_s_in->area_square,1);
	uint8_2_float(read_buffer+33,&p_s_in->cos_square,1);
	uint8_2_uint16(read_buffer+37,&p_s_in->thresh_diff2,1);
	uint8_2_float(read_buffer+41,&p_s_in->thresh_ratio2,1);
}
