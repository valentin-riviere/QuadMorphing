/*
This file is part of QuadMorphing project
Copyright (C) 2018 - Institute of Movement Sciences (Marseille, FRANCE)

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "Def.h"
#include "Regulation.h"

/*********************************** GLOBAL VARIABLES **********************************/

// Not useful if a and b lookUpTable is completed (don't forget to comment the call of InitLookUpTable during initialization)
volatile double Y_LookUpTable_FF[SIZE_LOOKUP_FF];		// [us] the time_up PWM i corresponding to the open-loop freq i

volatile double X_LookUpTable_FF[SIZE_LOOKUP_FF] = OMEGA_SP_HZ;  		// [us] the freq i corresponding to the open-loop PWM i
volatile double a_LookUpTable_FF[SIZE_LOOKUP_FF-1] = A_HZ_TO_US;
volatile double b_LookUpTable_FF[SIZE_LOOKUP_FF-1] = b_HZ_TO_US;

/******************************** END GLOBALE VARIABLES ********************************/

void initRegul(void)
{
	uint8_t i;
	
	for(i=0; i<NB_ROTOR; i++)
	{ 
		RegulSig.RotorSpeed_SetPoint[i] = 0.0;
		RegulSig.Measured_RotorSpeed[i] = 0.0;
		RegulSig.TimeUp_PWM_ESC[i] = 0.0;
		RegulSig.RotorStatus[i] = NOT_ARMED;
		RegulSig.ESCStatus[i] = NOT_ARMED;	
		
		// initialize structure with default parameters
		Pid[i].G_SatMin		= 1120.0;
		Pid[i].G_SatMax		= 2000.0;
		Pid[i].I_SatMin		= -100.0;
		Pid[i].I_SatMax		= 800.0;
	}

	Pid[ROTOR_1].kp 	= 40.0;
	Pid[ROTOR_1].ki		= 30.0;
	Pid[ROTOR_1].kaw 	= 30.0;	
	
	Pid[ROTOR_2].kp 	= 40.0;
	Pid[ROTOR_2].ki		= 30.0;
	Pid[ROTOR_2].kaw 	= 30.0;
	
	Pid[ROTOR_3].kp 	= 40.0;
	Pid[ROTOR_3].ki		= 30.0;
	Pid[ROTOR_3].kaw 	= 30.0;
	
	Pid[ROTOR_4].kp 	= 40.0;
	Pid[ROTOR_4].ki		= 30.0;
	Pid[ROTOR_4].kaw 	= 30.0;
	
	InitSetPointFilter();
	InitMeasureFilter();
	
	// InitLookUpTable();
	
	RegulSig.RegulState = REGUL_STATUS;
	
	switch (RegulSig.RegulState)
	{
		case OPEN_LOOP:
		{
			Serial.println("Regulation Mode: open_loop");
			break;
		}
		case CLOSED_LOOP:
		{
			Serial.println("Regulation Mode: closed_loop");
			break;
		}
		case IDLE:
		{
			Serial.println("Regulation Mode: idle");
			break;
		}
	}
}

float us2freq_table(float SetPoint_us)
{
	if(SetPoint_us > (float) MAXTHROTTLE)
	{
		return(0.0);
	}
	else if(SetPoint_us >= (float) MINTHROTTLE)
	{
		return(US_2_FREQ_a*SetPoint_us-US_2_FREQ_b);
	}
	else
	{
		return(0.0);
	}
}

void StartRotor(uint8_t RotorNum)
{
	char str[100];

	if (RegulSig.ESCStatus[RotorNum] != ARMED)
	{
		StartESC(RotorNum);
	}
	else // ESC ARMED
	{
		if (RegulSig.RotorStatus[RotorNum] == NOT_ARMED)
		{
			// apply pwm to arm motors
			RegulSig.TimeUp_PWM_ESC[RotorNum] = ARM_ROTOR_PWM;		
			
			sprintf((char*)str, "Rotor %d Arming with %d...\n", RotorNum+1, ARM_ROTOR_PWM);
			Serial.print(str);
			
			StepTime += ROTOR_INIT_TIME_us/4;	// Wait start rotor
			
			RegulSig.RotorStatus[RotorNum] = ARMED;
		}
	}
}

void StopRotor(uint8_t RotorNum)
{
	char str[20];
	
	if(RegulSig.RotorStatus[RotorNum] == ARMED)
	{
		// apply pwm to disarm rotor
		RegulSig.TimeUp_PWM_ESC[RotorNum] = ARM_ESC_PWM;	
		// mark that the rotor as stopped
		RegulSig.RotorStatus[RotorNum] = NOT_ARMED;
		
		sprintf((char*)str, "Rotor %u Stopped!", RotorNum+1);
		Serial.println(str);
	}
	else if(RegulSig.ESCStatus[RotorNum] == NOT_ARMED)
	{
		StartESC(RotorNum);
	}
	else
	{
		// Nothing
	}
}

void StartESC(uint8_t ESCNum)
{
	char str[50];
	
	// apply pwm to arm motors
	RegulSig.TimeUp_PWM_ESC[ESCNum] = ARM_ESC_PWM;

	sprintf((char*)str, "ESC %d Arming with %d...\n", ESCNum+1, ARM_ESC_PWM);
	Serial.print(str);
	
	StepTime += ESC_INIT_TIME_us/4;		// Wait init ESC
	
	RegulSig.ESCStatus[ESCNum] = ARMED;
}

void InitSetPointFilter(void)
{
	float z0 = ((float)((uint32_t)(exp(-2.0*PI*LPF_FREQUENCY*SAMPLE_TIME_s)*1000.0)))/1000.0;
	uint8_t  RotorNum, j;
	
	for(RotorNum=0; RotorNum<NB_ROTOR; RotorNum++)
	{
		for (j=0; j<FILTER_ORDER; j++)
		{				
			FilterSig[RotorNum].FilterStateHistory[j] = 0.0;
			FilterSig[RotorNum].FilterStateHistory[j] = 0.0;
		}
		
		FilterSig[RotorNum].a0_order1 = 1.0-z0;
		FilterSig[RotorNum].a1_order1 = z0;
		FilterSig[RotorNum].a0_order2 = (1.0-z0)*(1.0-z0);
		FilterSig[RotorNum].a1_order2 = 2.0*z0;
		FilterSig[RotorNum].a2_order2 = z0*z0;
	}
}

void InitMeasureFilter(void)
{
	float z0 = exp(-2.0*PI*LPF_FREQUENCY_MEAS*SAMPLE_TIME_s);
	uint8_t  RotorNum, j;
	
	for(RotorNum=0; RotorNum<NB_ROTOR; RotorNum++)
	{
		for (j=0; j<FILTER_ORDER; j++)
		{				
			FilterMeas[RotorNum].FilterStateHistory[j] = 0.0;
			FilterMeas[RotorNum].FilterStateHistory[j] = 0.0;
		}
		
		FilterMeas[RotorNum].a0_order1 = 1.0-z0;
		FilterMeas[RotorNum].a1_order1 = z0;
		FilterMeas[RotorNum].a0_order2 = (1.0-z0)*(1.0-z0);
		FilterMeas[RotorNum].a1_order2 = 2.0*z0;
		FilterMeas[RotorNum].a2_order2 = z0*z0;
	}
}

void InitLookUpTable(void)
{
	uint8_t i;
	
	for (i=0; i<(SIZE_LOOKUP_FF-1); i++)
	{
		a_LookUpTable_FF[i] = (Y_LookUpTable_FF[i+1]-Y_LookUpTable_FF[i])/(X_LookUpTable_FF[i+1]-X_LookUpTable_FF[i]);
		b_LookUpTable_FF[i] = Y_LookUpTable_FF[i] - a_LookUpTable_FF[i]*X_LookUpTable_FF[i];
	}
}

void SetPointFilter(uint8_t RotorNum)
{	
	uint8_t i;
	
	if (FILTER_ORDER == 0)
	{
		FilterSig[RotorNum].RotorSpeed_FilteredSetPoint = RegulSig.RotorSpeed_SetPoint[RotorNum];
	}
	else
	{
		// up to date the data history (euler backward)
		FilterSig[RotorNum].InputHistory[0] = RegulSig.RotorSpeed_SetPoint[RotorNum];	// u(k)
		
		// up to date the output of filter
		if (FILTER_ORDER == 1)
		{
			FilterSig[RotorNum].RotorSpeed_FilteredSetPoint = FilterSig[RotorNum].a0_order1*FilterSig[RotorNum].InputHistory[0]
												            + FilterSig[RotorNum].a1_order1*FilterSig[RotorNum].FilterStateHistory[0];
		}
		else
		{
			FilterSig[RotorNum].RotorSpeed_FilteredSetPoint = FilterSig[RotorNum].a0_order2*FilterSig[RotorNum].InputHistory[0]
														    + FilterSig[RotorNum].a1_order2*FilterSig[RotorNum].FilterStateHistory[0] - FilterSig[RotorNum].a2_order2*FilterSig[RotorNum].FilterStateHistory[1];
		}
		// up to date the data history 
		for (i=FILTER_ORDER-1; i>0; i--)
		{					// y(k-i-1) = y(k-i)
			FilterSig[RotorNum].FilterStateHistory[i] = FilterSig[RotorNum].FilterStateHistory[i-1];
		}
		FilterSig[RotorNum].FilterStateHistory[0] = FilterSig[RotorNum].RotorSpeed_FilteredSetPoint;
	}	
}

// FreqSetPoint (in Hz)
// Return PWM (in us)
float ComputeFeedForward(float FreqSetPoint)
{
	uint8_t Index;
	uint8_t Begin = 0;
	uint8_t End   = SIZE_LOOKUP_FF-1;
	
	// use a dichotomy algorithm to determine the part of the look-up table to use
	while(Begin!=End-1)
	{
		Index = (Begin+End)/2;
		if ( FreqSetPoint>=X_LookUpTable_FF[Index] )
		{
			Begin = Index;
		}
		else
		{
			End = Index;
		}
	}
	// Compute the FeedForwardTerm
	return(a_LookUpTable_FF[Index]*FreqSetPoint+b_LookUpTable_FF[Index]);
}

void RegulRotorSpeed(uint8_t RotorNum)
{
	static float PWM[NB_ROTOR] = {0.0, 0.0, 0.0, 0.0};
	static uint8_t NbMissedMagnet[NB_ROTOR] = {0, 0, 0, 0};
	static float DeltaSat[NB_ROTOR] = {0.0, 0.0, 0.0, 0.0};
	float FeedForwardPWM[NB_ROTOR];
	float PWM_Sat[NB_ROTOR] = {0.0, 0.0, 0.0, 0.0};
	float error_p;	
	char str[100];

	RegulSig.Measured_RotorSpeed[RotorNum] = Saturate(RegulSig.Measured_RotorSpeed[RotorNum], MIN_FREQ_SAT, MAX_FREQ_SAT);
	
	// to avoid wrong PWM when a magnet is missed
	if( (RegulSig.Measured_RotorSpeed[RotorNum] == (float)MIN_FREQ_SAT) && (RegulSig.RotorStatus[RotorNum] == ARMED) && (NbMissedMagnet[RotorNum] < 10) && (RotorNum == 0))
	{
		NbMissedMagnet[RotorNum]++;
		
		sprintf((char*)str, "Rotor %d: Missed magnet  %8.1f [Hz], %d magnets missed", RotorNum, (double)(RegulSig.Measured_RotorSpeed[RotorNum]), NbMissedMagnet[RotorNum]);
		Serial.println(str);
	}
	else
	{
		NbMissedMagnet[RotorNum] = 0;
		// filter the setpoint
		SetPointFilter(RotorNum); // not filtered if FILTER_ORDER == 0
		// ep(t) = kp*(SP(t) - MEAS(t))
		error_p = Pid[RotorNum].kp * (FilterSig[RotorNum].RotorSpeed_FilteredSetPoint - RegulSig.Measured_RotorSpeed[RotorNum]);	
		
		if(DeltaSat[RotorNum]>-2.0)
		{
			// I(t) = ki * ( (ep(t) - kaw*DeltaSat(t-1))*Ts ) + I(t-1)
			Pid[RotorNum].Integrator = Pid[RotorNum].ki * SAMPLE_TIME_s * (error_p - Pid[RotorNum].kaw * DeltaSat[RotorNum]) + Pid[RotorNum].Integrator;
		}
					
		// saturate the integrator
		Pid[RotorNum].Integrator = Saturate(Pid[RotorNum].Integrator, Pid[RotorNum].I_SatMin, Pid[RotorNum].I_SatMax);
		
		// PWM(t) = ep(t) + I(t)
		PWM[RotorNum] = error_p + Pid[RotorNum].Integrator;
		// staturate the PWM
		PWM_Sat[RotorNum] = Saturate(PWM[RotorNum], Pid[RotorNum].I_SatMin, Pid[RotorNum].I_SatMax);
		// DeltaSat(t) = PWM(t-1) - PWM_sat(t-1)
		DeltaSat[RotorNum] = PWM[RotorNum] - PWM_Sat[RotorNum];
		
		FeedForwardPWM[RotorNum] = Saturate(ComputeFeedForward(RegulSig.RotorSpeed_SetPoint[RotorNum]), MIN_TIMEUP_PWM, MAX_TIMEUP_PWM);
		#ifdef DEBUG_REGUL
			Debug1[RotorNum] = (uint16_T)(FilterSig[RotorNum].RotorSpeed_FilteredSetPoint*160.0);
			Debug2[RotorNum] = (uint16_T)(DeltaSat[RotorNum]*us_2_OC);
			Debug3[RotorNum] = (uint16_T)(Pid[RotorNum].Integrator*us_2_OC+32768.0);
		#endif
		
		// PWM_sat(t) = sat(PWM(t), PWM_MIN, PWM_MAX)
		RegulSig.TimeUp_PWM_ESC[RotorNum] = Saturate(PWM_Sat[RotorNum] + FeedForwardPWM[RotorNum], Pid[RotorNum].G_SatMin, Pid[RotorNum].G_SatMax);
		DeltaSat[RotorNum] +=  (PWM_Sat[RotorNum] + FeedForwardPWM[RotorNum]) - RegulSig.TimeUp_PWM_ESC[RotorNum];
	}
}