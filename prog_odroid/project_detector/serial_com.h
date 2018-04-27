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
#ifndef SERIAL_COM_H
#define SERIAL_COM_H

#include <iostream>
#include <errno.h>
#include <signal.h>
#include <chrono>
#include <semaphore.h>
#include "types_convert.h"
#include "class_def.h"
#include "serial_lib.h"

using namespace std;

#define WAIT_CONNECTION_TIME 100000	// In us, time between two read buffer for asking connection
#define POLLING_TIME_MS 10	// Polling time for loop
#define HEADER 170
#define NB_BYTE_TO_READ 48
#define NB_BYTE_TO_WRITE 28
#define NB_GUMSTIX_FAILED 3		// Number of failing read before re-initialization

// Serial communication
int serial_com(Stream_in * p_s_in, const Stream_out * p_s_out, bool * p_sh_start, sem_t* sem);

// Convert read bytes in stream_in data format
void convert_stream_in(const uint8_t * read_buffer, Stream_in * p_s_in);

#endif
