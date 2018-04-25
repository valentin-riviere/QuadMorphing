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
#ifndef SERIAL_LIB_H
#define SERIAL_LIB_H

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>

#define SERIAL_BAUDRATE B115200
#define TIME_PER_BYTE 87 // Time in us to transmit one byte in UART (8+2 = 10bits) with specified baudrate
#define START_STREAM "START"

extern int fd;	// Socket for Serial
extern struct termios SerialPortSettings; // Serial struct configuration

int open_serial_port(const char* port);
int8_t close_serial_port(void);
int8_t flush_buffers(void);
int8_t start_asked(void);
int8_t send_start_serial(void);
// Read and write functions on serial port (NbByte < 255)
int8_t write_serial(const uint8_t* str, const uint8_t NbByteToWrite);
int8_t header_write_serial(const uint8_t* str, const uint8_t NbByteToWrite, const uint8_t header);
int8_t read_serial(uint8_t* str, const uint8_t ByteToRead);
int8_t header_read_serial(uint8_t* str, const uint8_t ByteToRead, const uint8_t header);

#ifdef __cplusplus
	} // End extern C
#endif

#endif
