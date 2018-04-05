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
#include "serial_lib.h"


int fd;	// Socket for Serial
struct termios SerialPortSettings; // Serial struct configuration

// Open and configure Serial Port
int8_t open_serial_port(const char *port)
{
	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY); // O_NONBLOCK : Non-blocking mode, O_NDELAY : No delay mode for reading
	if (fd == -1)
		return -1;

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
	SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);	// Mode of operation (Non canonical mode)
	SerialPortSettings.c_oflag &= ~(OPOST);	// No output processing	
	SerialPortSettings.c_cc[VMIN] = 0;		// wait 0 characters before read returning (no effect with NONBLOCK and NDELAY FLAG)
	SerialPortSettings.c_cc[VTIME] = 0;		// Wait indefinitely (no effect with NONBLOCK and NDELAY FLAG)
	tcflush(fd,TCIFLUSH);					// Flush RX buffer

	if (tcsetattr(fd,TCSANOW,&SerialPortSettings) != 0)	// Save configuration
		return -1;

	return 0;
}

// Close device
int8_t close_serial_port(void)
{
	return close(fd);
}

int8_t start_asked(void)
{
	uint8_t read_buffer[255], start_str[] = START_STREAM;

	if (read_serial(read_buffer,sizeof(START_STREAM)-1) != sizeof(START_STREAM)-1)
		return 0;
	else
	{
		for (uint8_t i ; i < sizeof(START_STREAM)-1 ; i++)
		{
			if (read_buffer[i] != start_str[i])
				return 0;
		}
	
		return 1;
	}
}

// Send start stream
int8_t send_start_serial(void)
{
	uint8_t buf_write[255];

	memcpy(buf_write,START_STREAM,sizeof(START_STREAM)-1);
	return write_serial(buf_write,sizeof(START_STREAM)-1);
}

int8_t write_serial(const uint8_t* write_buffer, const uint8_t NbByteToWrite)
{
	int bytes_written = 0;

	bytes_written = write(fd,write_buffer,NbByteToWrite);
	if (bytes_written < 0)
		return -1;

	return bytes_written;
}

int8_t header_write_serial(const uint8_t* write_buffer, const uint8_t NbByteToWrite, const uint8_t header)
{
	int bytes_written = 0;

	write(fd,&header,1);
	bytes_written = write(fd,write_buffer,NbByteToWrite);
	if (bytes_written < 0)
		return -1;

	return bytes_written;
}

int8_t read_serial(uint8_t* read_buffer, const uint8_t NbByteToRead)
{
	int bytes_read = 0;

	bytes_read = read(fd,read_buffer,NbByteToRead);
	if (bytes_read < 0)
		return -1;

	return bytes_read;
}

int8_t header_read_serial(uint8_t* read_buffer, const uint8_t NbByteToRead, const uint8_t header)
{
	int bytes_read = 0;
	
	do	// Read one time and loop if read != header
	{
		bytes_read = read(fd,read_buffer,1);
		if (bytes_read < 0)
			return -1;
		else
			usleep(TIME_PER_BYTE);	// Wait for next byte
	}while (read_buffer[0] != header);

	usleep(TIME_PER_BYTE*NbByteToRead);	// Wait for tram acquisition

	bytes_read = read(fd,read_buffer,NbByteToRead);
	if (bytes_read != NbByteToRead)	// If bytes_read != NbByteToRead
		return 0;

	return bytes_read;
}
