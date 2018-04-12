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
#include <iostream>
#include <errno.h>
#include <signal.h>
#include <chrono>
#include "serial_lib.h"
#include "types_convert.h"

using namespace std;

// Debug with RX/TX connected
//#define AUTO_DEBUG

#define WAIT_CONNECTION_TIME 100000	// In us, time between two read buffer for asking connection
#define POLLING_TIME_MS 10	// Polling time for loop
#define HEADER 170
#define NB_BYTE_TO_READ 43
#define NB_GUMSTIX_FAILED 3		// Number of failing read before re-initialization

bool loop_on = true;	// Variable for program loop

void sig_handler(int signum)
{
	cout << "Quit serial Gumstix Program" << endl;
	loop_on = false;
}

int main(void)
{
	bool socket_on = false;	// Connection between odroid and RCB 2 is off
	int bytes_written = 0, bytes_read = 0, gumstix_fail = 0;
	uint8_t read_buffer[255];
	const chrono::milliseconds Duration_com(POLLING_TIME_MS); // Polling time for communication
	chrono::high_resolution_clock::time_point cTime = chrono::high_resolution_clock::now(), pTime = chrono::high_resolution_clock::now();

	// singal interrupts
	signal(SIGINT, sig_handler);	// CTRL C
	signal(SIGTERM, sig_handler);	// Terminate
	signal(SIGHUP, sig_handler);	// Close terminal

	// Open and configure Serial Port
	if (open_serial_port("/dev/ttySAC0"))
	{
		cout << "Error with open and configure serial port: " << strerror(errno) << endl;
		return -1;
	}

#ifdef AUTO_DEBUG
	// Send start as Gumstix
	send_start_serial();
#endif
	
	// Program loop
	while (loop_on)
	{
		cTime = chrono::high_resolution_clock::now();
		if(chrono::duration_cast<chrono::milliseconds>(cTime - pTime) >= Duration_com)
		{
			pTime = cTime;
			if (!socket_on)	// If socket is not initialized
			{
				if (start_asked()) // If start received and connection is off
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
				// Convert in uint8
	//			float f_send[] = {3.14, -5e3}, f_read[2];
				uint8_t write_buffer[26]; // = {1,2,3,4};
				for (uint8_t i = 0 ; i < sizeof(write_buffer) ; i++)
					write_buffer[i] = i;
	//			float_2_uint8(f_send,write_buffer,2);

				// Write on Serial Port
				bytes_written = header_write_serial(write_buffer,sizeof(write_buffer),HEADER);
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
				}
				else	// read_buffer contains gumstix data
				{
					gumstix_fail = 0;	// Reset gumstix fails when receive something
					cout << bytes_read << " bytes read!" << endl;
					for (uint8_t i = 0 ; i < bytes_read ; i++)
						printf("%x ",read_buffer[i]);
					cout << endl;
	//				uint8_2_float(read_buffer, f_read, 2);
	//				for (uint8_t i = 0 ; i < sizeof(f_read)/4 ; i++)
	//					printf("%f ",f_read[i]);
	//				cout << endl;
				}
			}
		}
	}

	close_serial_port();

	return 0;
}
