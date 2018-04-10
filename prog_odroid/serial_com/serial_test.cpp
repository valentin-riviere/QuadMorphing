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
#include <stdio.h>
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include "types_convert.h"

using namespace std;

#define SERIAL_BAUDRATE B115200

int main(void)
{
	int fd;
	struct termios SerialPortSettings;

	// Open Serial Port
	fd = open("/dev/ttySAC0", O_RDWR | O_NOCTTY | O_NDELAY); // O_NONBLOCK : Non-blocking mode, O_NDELAY : No delay mode for reading
	if (fd == -1)
	{
		cout << "Error! in Opening ttyS0" << endl;
		return -1;
	}
	else
		cout << "ttyS0 Opened Successfully" << endl;

	// Configure Serial Port
	tcgetattr(fd, &SerialPortSettings);
	cfsetispeed(&SerialPortSettings,SERIAL_BAUDRATE);	// Set baudrate input
	cfsetospeed(&SerialPortSettings,SERIAL_BAUDRATE);	// Set baudrate output
	SerialPortSettings.c_cflag &= ~PARENB;	// No parity bit
	SerialPortSettings.c_cflag &= ~CSTOPB;	// Stop bit = 1
	SerialPortSettings.c_cflag &= ~CSIZE;	// Clear mask
	SerialPortSettings.c_cflag |= CS8;		// Data bits = 8
	SerialPortSettings.c_cflag &= ~CRTSCTS;	// Turn off HW based flow ctrl
	SerialPortSettings.c_cflag |= CREAD | CLOCAL;	// Turn on receiver Serial port
	SerialPortSettings.c_iflag &= ~(IGNPAR | IXON | IXOFF | IXANY);	// Turn off flow control
	SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);	// Mode of operation
	SerialPortSettings.c_oflag &= ~(OPOST);	// No output processing	
	SerialPortSettings.c_cc[VMIN] = 0;		// wait 0 characters before read returning (no effect with NONBLOCK and NDELAY FLAG)
	SerialPortSettings.c_cc[VTIME] = 0;		// Wait indefinitely (no effect with NONBLOCK and NDELAY FLAG)
	tcflush(fd,TCIFLUSH);	// Flush RX buffer
	if (tcsetattr(fd,TCSANOW,&SerialPortSettings) != 0)	// Save configuration
	{
		cout << "Error! in setting atributes" << endl;
		return -1;
	}

	// Write on Serial Port
	float write_float[] = {3.14, -5.2e3};
//	char write_buffer[] = "TU_LE_SAIS";
	uint8_t write_buffer[8]; float_2_uint8(write_float,write_buffer,2);
	unsigned char header = 170;
	int bytes_written = 0;

	// Write header
	write(fd,&header,1);
//	bytes_written = write(fd,write_buffer,sizeof(write_buffer)-1);
	bytes_written = write(fd,write_buffer,sizeof(write_buffer));
	if (bytes_written < 0)
		cout << "UART TX error" << endl;

	// Read on Serial Port
	uint8_t read_buffer[255];
	int bytes_read = 0, NbByteToRead=10; read_buffer[NbByteToRead+1]='\0';

	do	// Wait read buffer = NbByteToRead
	{
		bytes_read = read(fd,&read_buffer,NbByteToRead+1);
		if (bytes_read < 0)
		{
			cout << "UART RX error: " << strerror(errno) << endl;
			return -1;
		}
	}while(bytes_read <= 0);

	if (read_buffer[0] == header)
	{
		float out[2];
		uint8_2_float(&(read_buffer[1]),out,2);
		cout << "1st : " << out[0] << endl;
		cout << "2nd : " << out[1] << endl;
	}

	// Close device
	close(fd);
	cout << "ttyS0 Closed Successfully" << endl;

	return 0;
}
