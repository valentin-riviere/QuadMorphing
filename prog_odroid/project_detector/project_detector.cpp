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
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/mman.h>
#include "def.h"
#include "detection.h"
#include "serial_com.h"

using namespace cv;
using namespace Pylon;
using namespace std;

bool loop_main = true;

void sig_handler(int signum)
{
	cout << "Quit project_detector Program" << endl;
	loop_main = false;
}

int main(int argc, char* argv[])
{
	// singal interrupts
	signal(SIGINT, sig_handler);	// CTRL C
	signal(SIGTERM, sig_handler);	// Terminate
	signal(SIGHUP, sig_handler);	// Close terminal

    int16_t exitCode = 0; 	// The exit code
	pid_t pid;
	uint8_t com_start = 0;	// To synchronize detection and communication

	// Create shared memory
	Stream_in* p_shs_in = (Stream_in*) mmap(NULL, sizeof(Stream_in), PROT_READ | PROT_WRITE, MAP_ANONYMOUS | MAP_SHARED, 0, 0);
	Stream_out* p_shs_out = (Stream_out*) mmap(NULL, sizeof(Stream_out), PROT_READ | PROT_WRITE, MAP_ANONYMOUS | MAP_SHARED, 0, 0);
	uint8_t* p_shstart = (uint8_t*) mmap(NULL, sizeof(com_start), PROT_READ | PROT_WRITE, MAP_ANONYMOUS | MAP_SHARED, 0, 0);

	// Initialize shared memory
	memcpy(p_shstart,&com_start,sizeof(com_start));
//	memcpy(shs_in,&s_in,sizeof(s_in));
	
	// Fork
	pid = fork();
	if (pid < 0)
	{
		cout << "Error with fork : " << strerror(errno) << endl;
		return -1;
	}
	else if (pid == 0) // Child process
	{
		detection(p_shs_in, p_shs_out, p_shstart);
	}
	else	// Parent process
	{
		serial_com(p_shs_in, p_shs_out, p_shstart);
	}

    return exitCode;
}
