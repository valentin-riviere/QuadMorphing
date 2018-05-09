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
	f << "Stream_in : " << endl << p_s_in->t_poll << endl << p_s_in->max_num_buffer << endl << p_s_in->width;
	return f;
}

int serial_com(Stream_in * p_s_in, const Stream_out * p_s_out, bool * p_sh_start, sem_t * sem)
{
	bool socket_on = false;	// Connection between odroid and RCB 2 is off
	int bytes_written = 0, bytes_read = 0, gumstix_fail = 0;
	uint8_t read_buffer[NB_BYTE_TO_READ], write_buffer[NB_BYTE_TO_WRITE] = {0};
	const chrono::milliseconds Duration_com(T_COM_MS); // Polling time for communication
	chrono::high_resolution_clock::time_point cTime = chrono::high_resolution_clock::now(), pTime = chrono::high_resolution_clock::now(), pTime_print = chrono::high_resolution_clock::now();
	uint32_t overrun = 0;	// Nbr of overruns
	uint32_t ticks = 0;		// Nbr of ticks in loop

	// Open and configure Serial Port
	if (open_serial_port("/dev/ttySAC0") < 0)
	{
		cout << "Error with open and configure serial port: " << strerror(errno) << endl;
		return -1;
	}
	
	// Program loop
	while (loop_main)
	{
		cTime = chrono::high_resolution_clock::now();

		chrono::milliseconds duration = chrono::duration_cast<chrono::milliseconds>(cTime - pTime);
		if(duration >= Duration_com)
		{
			// Reinitialize timer
			pTime = cTime;

#ifdef PRINT_DEBUG
			if (duration > Duration_com && socket_on)
			{
				overrun++;
			}
#endif

			if (!socket_on)	// If socket is not initialized
			{
				sem_wait(sem);				// Critical section
					*p_sh_start = false;	// Reset detection if socket is off
				sem_post(sem);

				if (start_asked()) 	// If start received and connection is off
				{
					// Send start stream on Serial Port
					cout << "Received START stream..." << endl;
					send_start_serial();
					socket_on = true;
					gumstix_fail = 0;
					cout << "Connection established!" << endl;
					usleep(10000);
				}
				else	// Wait until next pool
				{
					cout << "Wait START stream and flush buffers..." << endl;
					flush_buffers();				// Flush RX/TX buffer
					usleep(WAIT_CONNECTION_TIME);
				}
			}

			if (socket_on)	// Socket is initialized
			{
				// Write on Serial Port
				sem_wait(sem); // Critical section
					p_s_out->to_buffer(write_buffer);
				sem_post(sem);
				bytes_written = header_write_serial(write_buffer,NB_BYTE_TO_WRITE,HEADER);
				if (bytes_written < 0)	// If troubles, exit program
				{
					cout << "UART TX error: " << strerror(errno) << endl;
					loop_main = false;
				}

				// Read on Serial Port
				bytes_read = header_wait_read_serial(read_buffer,NB_BYTE_TO_READ,HEADER);
				if (bytes_read < 0)		// If troubles, exit program
				{
					cout << "UART RX error: " << strerror(errno) << endl;
					loop_main = false;
				}
				else if (bytes_read != NB_BYTE_TO_READ)	// If nothing to read => increment gumstix fails
				{
					if (++gumstix_fail >= NB_GUMSTIX_FAILED)
						socket_on = false;
					cout << "No stream received " << gumstix_fail << " / " << NB_GUMSTIX_FAILED;
					cout << "\t" << bytes_read << " bytes read! (" << NB_BYTE_TO_READ << " bytes expected)" << endl;
				}
				else	// read_buffer contains gumstix data
				{
					gumstix_fail = 0;	// Reset gumstix fails when receive something
#ifdef DEBUG_SERIAL
					cout << "Received bytes :\n";
					for (unsigned int i = 0 ; i < NB_BYTE_TO_READ ; i++)
						printf("%d\n", (uint16_t) read_buffer[i], read_buffer[i]);
#endif
					sem_wait(sem);	// Critical section
						p_s_in->convert_from_buffer(read_buffer);	// Convert bytes to struct stream_in data format
						*p_sh_start = true;							// Detection is set
					sem_post(sem);
				}

				ticks++;
			}
		}
#ifdef PRINT_DEBUG
		else if (chrono::duration_cast<chrono::milliseconds>(cTime - pTime_print) >= (chrono::milliseconds) T_PRINT)
		{
			pTime_print = cTime;
			cout << "Communication Task (" << Duration_com.count() << "ms)" << endl;
			cout << "\t\t" << "OverRuns : " << overrun << "/" << ticks << " ticks" << endl;
		}
#endif
	} // End While

	close_serial_port();

	return 0;
}
