/*
This file is part of QuadMorphing project
Copyright (C) 2018 - Institute of Movement Sciences (Marseille, FRANCE)

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "Def.h"

extern float gyroADC[3];
extern float accADC[3];
extern float magADC[3];

void Pilot_loop()
{
	uint32_t TestTime;

	if(ManualMode == 0)
	{
		/*********** INITIALISATION TEST ***********
		If initialisation is NOT done
		 *******************************************/
		if (init_OK <= 0) // if initialisation isn't done
		{
			/*********** TEST StartStream *************  
			If start stream was received from Gumstix 
				=> 1) Send start stream answer 
				   2) Execute onestep of Gumstix slave
			*******************************************/
			if (StartAsked()) 
			{
				GumstixPilot_FirstStep();
			}
			/*********** TEST StartStream *************  
			if start stream was NOT received from Gumstix 
				=> 1) AutoPilot(): 
						 - Keep at the current state (flying or inactive)
						 - if flying: 
							 => stay at the current position (IMU estimate)
			*******************************************/  
			else
			{
				#ifdef ECHO_TIME_EXEC_PILOT
					TestTime = micros();
				#endif

				AutoPilot();
				
				#ifdef ECHO_TIME_EXEC_PILOT
					int32_t t_exec_ap = micros()-TestTime;
					if (t_exec_ap_max < t_exec_ap)
					t_exec_ap_max = t_exec_ap;
					Serial.print("\nAutoPilot: ");
					Serial.println(t_exec_ap);
					Serial.print("AutoPilot Max: ");
					Serial.println(t_exec_ap_max);
				#endif

				#ifdef ECHO_PRINTF
					Display_angle_motors();
				#endif
			}
		}
		/*********** INITIALISATION TEST ***********
		If initialisation is done:
			 => Execute onestep of Gumstix slave
				1) Send Sensor
				2) Read motor and up to date motor
		 *******************************************/
		else
		{    
			#ifdef ECHO_TIME_EXEC_PILOT
				TestTime = micros();
			#endif

			GumstixPilot_OneStep();

			#ifdef ECHO_TIME_EXEC_PILOT
				int32_t t_exec_gum = micros()-TestTime;
				if (t_exec_gum_max < t_exec_gum)
				  t_exec_gum_max = t_exec_gum;
				Serial.print("\nGumstixPilot: ");
				Serial.println(t_exec_gum);
				Serial.print("GumstixPilot Max: ");
				Serial.println(t_exec_gum_max);
			#endif
		}
	}
	else
	{
		#ifdef ECHO_TIME_EXEC_PILOT
			TestTime = micros();
		#endif
		  
		RxPilot();

		#ifdef ECHO_TIME_EXEC_PILOT
			int32_t t_exec = micros()-TestTime;
			if (t_exec_rx_pilot_max < t_exec)
			t_exec_rx_pilot_max = t_exec;
			Serial.print("\nRxPilot: ");
			Serial.println(t_exec);
			Serial.print("RxPilot max: ");
			Serial.println(t_exec_rx_pilot_max);
		#endif
	}
}

void RxPilot(void)
{
	// recover IMU values
	IMU_getADC_full();

	// estimate attitude
	EstimateAttitude();
	
	// CompuetRxAttitudeSetPoint
	ComputeEulerSetPoint();

	// Compute motor reference
	StabilizeAttitude();

	//if (rcData[THROTTLE] < 1050)
	if (MotorsArmed == 0)
	{
		WriteAllMotors(MINCOMMAND);
	}
	else
	{  
		if (rcData[THROTTLE] < 1100)
		{
			WriteAllMotors(MINTHROTTLE);
		}
		else
		{
			// Apply motor's values
			WriteMotors();
		}
	}
}

void AutoPilot(void)
{
#ifdef ECHO_TIME_EXEC_AUTOPILOT
	int32_t t_exec;
	uint32_t TestTime = micros();
#endif

	// recover IMU values
	IMU_getADC_full();

#ifdef ECHO_TIME_EXEC_AUTOPILOT
	t_exec = micros()-TestTime;
	Serial.print("\nIMU_getADC_full(): ");
	Serial.println(t_exec);
#endif

#ifdef ECHO_TIME_EXEC_AUTOPILOT
	TestTime = micros();
#endif

	// estimate attitude
	EstimateAttitude();

#ifdef ECHO_TIME_EXEC_AUTOPILOT
	t_exec = micros()-TestTime;
	Serial.print("\nEstimateAttitude(): ");
	Serial.println(t_exec);
#endif

#ifdef ECHO_TIME_EXEC_AUTOPILOT
	TestTime = micros();
#endif

	// CompuetRxAttitudeSetPoint
	ComputeEulerSetPoint();

#ifdef ECHO_TIME_EXEC_AUTOPILOT
	t_exec = micros()-TestTime;
	Serial.print("\nComputeEulerSetPoint(): ");
	Serial.println(t_exec);
#endif

	// enforce the throttle setpoint to just compensate the robot's weight
	ThrottleSetPoint = ROBOT_WEIGHT*9.81;

#ifdef ECHO_TIME_EXEC_AUTOPILOT
	TestTime = micros();
#endif

	// Compute motor reference
	StabilizeAttitude();

#ifdef ECHO_TIME_EXEC_AUTOPILOT
	t_exec = micros()-TestTime;
	Serial.print("\nStabilizeAttitude(): ");
	Serial.println(t_exec);
#endif

#ifdef ECHO_TIME_EXEC_AUTOPILOT
	TestTime = micros();
#endif

	//if (rcData[THROTTLE] < 1050)
	if (MotorsArmed == 0)
	{
		WriteAllMotors(MINCOMMAND);
	}
	else
	{  
		if (rcData[THROTTLE] < 1100)
		{
			WriteAllMotors(MINTHROTTLE);
		}
		else
		{
			// Apply motor's values
			WriteMotors();
		}
	}

#ifdef ECHO_TIME_EXEC_AUTOPILOT
	t_exec = micros()-TestTime;
	Serial.print("\nWriteMotors(): ");
	Serial.println(t_exec);
#endif
}

void GumstixPilot_FirstStep(void)
{
cli();
	SendStartStream_RS232();
sei();

	// recover IMU values
	IMU_getADC_full();

	// copy acc(in g)/ang_speed(in rad/s)/mag(in mG)/pressure(in mBar) values in DataToSend
#ifdef USE_TEENSY_CAL	// If Teensy calibration used
	accADC[0] = accADC[0] + accelBias[0];
	accADC[1] = accADC[1] + accelBias[1];
	accADC[2] = accADC[2] - accelBias[2];
#endif
	memcpy(DataToSend, accADC, 12);     	// copy 3*4bytes (3*float32)
	memcpy(&DataToSend[6], gyroADC, 12); 	// copy 3*4bytes (3*float32)
	memcpy(&DataToSend[12], magADC, 12); 	// copy 3*4bytes (3*float32)
	memcpy(&DataToSend[18], &Pressure, 8); 	// copy 2*4bytes (1*float64)

	// copy RX data in DataToSend
	memcpy(&DataToSend[22], rcData, 8);     // copy 4*2bytes (ROLL, PITCH, YAW, THROTTLE) 

	// copy Vbat in DataToSend
	DataToSend[26] = vbatRaw;

	// send data values
cli();
	#ifdef HEADER_RS232_ON
		HeaderSendnInt16_RS232(DataToSend, 27, 170);
	#else
		SendnInt16_RS232(DataToSend, 27); 
	#endif 
sei();
	//n_read = NB_DATA_TO_RECEIVE; // to avoid error with
	// motor are not read the first time because Gumstix has not computed the first command yet

	// initialisation done
	init_OK = NB_MISSED_GUMSTIX ; // 3 because if 3 read failed, activ autopilot
	n_offset = 0;

	// estimate attitude in case of gumstix failure
	EstimateAttitude();

	// TEST
	int16_t buff[5];
	WaitnInt16_RS232(buff,NB_DATA_TO_RECEIVE+1);
	
	Serial.println("  => GumstixPilot activated!");
}

void GumstixPilot_OneStep(void)
{
	int8_t n_read = 0;

	// recover IMU values
	IMU_getADC_full();

	// copy acc(in g)/ang_speed(in rad/s)/mag(in mG)/pressure(in mBar) values in DataToSend
	memcpy(DataToSend, accADC, 12);     	// copy 3*4bytes
	memcpy(&DataToSend[6], gyroADC, 12); 	// copy 3*4bytes
	memcpy(&DataToSend[12], magADC, 12); 	// copy 3*4bytes
	memcpy(&DataToSend[18], &Pressure, 8); 	// copy 2*4bytes

	// copy RX data in DataToSend
	memcpy(&DataToSend[22], rcData, 8);     // copy 4*2bytes (ROLL, PITCH, YAW, THROTTLE) 

	// copy Vbat in DataToSend
	DataToSend[26] = vbatRaw;

	// send data values
cli();
	#ifdef HEADER_RS232_ON
		HeaderSendnInt16_RS232(DataToSend, 27, 170);
	#else
		SendnInt16_RS232(DataToSend, 27); 
	#endif 
sei();

	// read motor setpoint
	#ifdef HEADER_RS232_ON
		n_read = HeaderReadnInt16_RS232(motor, NB_DATA_TO_RECEIVE, 170);
	#else
		n_read = ReadnInt16_RS232(motor, NB_DATA_TO_RECEIVE);
	#endif

	// up to date motor PWM
	// NB: if n_read < NB_MOTOR only the n_read first motor will be updated 
	// and previous value are kept for motor > n_read
	WriteMotors();

	if (n_read < NB_DATA_TO_RECEIVE)
	{
		n_offset = NB_DATA_TO_RECEIVE-n_read; // at tester avec += en cas de plusieur loupé...
		Serial.print("WARNING!!! only ");
		Serial.print(n_read);
		Serial.print("/");
		Serial.print(NB_DATA_TO_RECEIVE);
		Serial.println(" Data received from Gumstix!!!");
		if (n_read == 0)
		{
			init_OK--;
			n_offset = NB_DATA_TO_RECEIVE;
			
			if (init_OK == 0)
			{
				Serial.println("Gumstix unreachable...");
				Serial.println("  => AutoPilot activated!");
			}
			else
			{
				// nothing
			}
		}
		else
		{
			// nothing to do
		}
	}
	else
	{
		init_OK = NB_MISSED_GUMSTIX ;
		n_offset = 0;
	}

	// estimate attitude in case of gumstix failure
	EstimateAttitude();
}