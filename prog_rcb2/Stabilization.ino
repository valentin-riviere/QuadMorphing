/*
This file is part of QuadMorphing project
Copyright (C) 2018 - Institute of Movement Sciences (Marseille, FRANCE)

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "Def.h"

// RX SetPoints
#define MAX_ANGLE_SETPOINT 0.349065850398866f  // 20*PI/180
#define MIN_ANGLE_SETPOINT -0.349065850398866  // -20*PI/180
#define MAX_THROTTLE_SETPOINT  4.0  // m*g*1.5 = 0.307*9.81*1.35
#define MIN_THROTTLE_SETPOINT  0.0   // 0*m*g
#define RX_MAX  2000
#define RX_MIN  1000
#define RX_DEADZONE  10.0f
#define Dt_AttCtl 0.005f
#define b_PWM   754.5836543402008f
#define a_PWM   1.214176533723936f
#define OMEGA_ROTOR_SQUARE_MAX 1052120.0f
#define OMEGA_ROTOR_SQUARE_MIN 80932.0f

// CONTROLLER PARAMETERS
#define OMEGA_MAX_SETPOINT             3.490658503988659f  // 200*pi/180 [rad/sec]
#define DELTA_THRUST_MAX_SETPOINT      0.047088f           // 2*l*(cT*omega_r_max^2-cT*omega_r_min^2)/5  with omega_r_max = 1026 rad/sec (corresponding to 130.0g) and omega_r_min = 284 rad/sec (corresponding to 10.0g), cT in N/((rad/sec)^2)
#define DELTA_THRUST_MAX_SETPOINT_YAW  0.000366579943907f  // 2*(cQ*omega_r_max^2-cQ*omega_r_min^2)/5
#define PWM_MIN 1120
#define PWM_MAX 2000

static float Kp_Euler[3] = {6.0, 6.0, 6.0};
static float Kp_Omega[3] = {0.02, 0.018, 0.001};
static float Ki_Omega[3] = {0.2, 0.2, 0.018};

static float iGamma[4][4] = {	206249.5936883, -2062495.9368830, -2062495.9368830,  264932139.0601118, 
								206249.5936883,  2062495.9368830, -2062495.9368830, -264932139.0601118, 
								206249.5936883,  2062495.9368830,  2062495.9368830,  264932139.0601118,
								206249.5936883,  -2062495.9368830,  2062495.9368830, -264932139.0601118};

static float Rx_a_att = ((float)MAX_ANGLE_SETPOINT - (float)MIN_ANGLE_SETPOINT)/((float)RX_MAX - (float)RX_MIN -2.0*(float)RX_DEADZONE);
static float Rx_b = (float)RX_MIN + (float)(RX_MAX-RX_MIN)/2.0;
static float Rx_a_thr = (float)(MAX_THROTTLE_SETPOINT - MIN_THROTTLE_SETPOINT)/((float)(RX_MAX - RX_MIN));

static float Int_OmegaLoop[3] = {0.0, 0.0, 0.0};  // Integrator of omega loop's PI

void ComputeEulerSetPoint(void)
{
	// remove offset RX_MIN
	EulerSetPoint[ROLL]  = (float)rcData[ROLL]  - Rx_b;
	EulerSetPoint[PITCH] = (float)rcData[PITCH] - Rx_b;
	EulerSetPoint[YAW]   = (float)rcData[YAW]   - Rx_b;
	ThrottleSetPoint     = (float)rcData[THROTTLE] - RX_MIN;

	// apply deadzone
	EulerSetPoint[ROLL]  = DeadZone(EulerSetPoint[ROLL],  RX_DEADZONE);
	EulerSetPoint[PITCH] = DeadZone(EulerSetPoint[PITCH], RX_DEADZONE);
	EulerSetPoint[YAW]   = DeadZone(EulerSetPoint[YAW],   RX_DEADZONE);

	// Apply scale factor
	EulerSetPoint[ROLL]  = Rx_a_att*EulerSetPoint[ROLL];
	EulerSetPoint[PITCH] = Rx_a_att*EulerSetPoint[PITCH];
	EulerSetPoint[YAW]   = Rx_a_att*EulerSetPoint[YAW];
	ThrottleSetPoint = Rx_a_thr*ThrottleSetPoint;

	// Saturate value
	EulerSetPoint[ROLL]  = Saturate(EulerSetPoint[ROLL] , MIN_ANGLE_SETPOINT, MAX_ANGLE_SETPOINT);
	EulerSetPoint[PITCH] = Saturate(EulerSetPoint[PITCH], MIN_ANGLE_SETPOINT, MAX_ANGLE_SETPOINT);
	EulerSetPoint[YAW]   = Saturate(EulerSetPoint[YAW]  , MIN_ANGLE_SETPOINT, MAX_ANGLE_SETPOINT);
	ThrottleSetPoint     = Saturate(ThrottleSetPoint, MIN_THROTTLE_SETPOINT, MAX_THROTTLE_SETPOINT);
}

void StabilizeAttitude(void)
{
	float OmegaSetPoint[3];
	float ErrorOmega;
	float Torques[3];
	float Command[4];
	float Omega_Rotor[4] = {0.0, 0.0, 0.0, 0.0};
	int i, j;

	// Angles loop
	OmegaSetPoint[ROLL]  = Saturate( Kp_Euler[ROLL] *(EulerSetPoint[ROLL]  - EulerAngles[ROLL])  , -OMEGA_MAX_SETPOINT, OMEGA_MAX_SETPOINT);
	OmegaSetPoint[PITCH] = Saturate( Kp_Euler[PITCH]*(EulerSetPoint[PITCH] - EulerAngles[PITCH]) , -OMEGA_MAX_SETPOINT, OMEGA_MAX_SETPOINT);
	OmegaSetPoint[YAW]   = Saturate( Kp_Euler[YAW]  *(EulerSetPoint[YAW]   - EulerAngles[YAW])   , -OMEGA_MAX_SETPOINT, OMEGA_MAX_SETPOINT);

	// Omega loop
	// ROLL
	ErrorOmega = Kp_Omega[ROLL]*(OmegaSetPoint[ROLL] - Omega_hat[ROLL]);
	Int_OmegaLoop[ROLL] = Saturate( Int_OmegaLoop[ROLL] + Ki_Omega[ROLL]*ErrorOmega*Dt_AttCtl, -DELTA_THRUST_MAX_SETPOINT, DELTA_THRUST_MAX_SETPOINT);
	Torques[ROLL] = Saturate(ErrorOmega + Int_OmegaLoop[ROLL], -DELTA_THRUST_MAX_SETPOINT, DELTA_THRUST_MAX_SETPOINT);
	// PITCH
	ErrorOmega = Kp_Omega[PITCH]*(OmegaSetPoint[PITCH] - Omega_hat[PITCH]);
	Int_OmegaLoop[PITCH] = Saturate( Int_OmegaLoop[PITCH] + Ki_Omega[PITCH]*ErrorOmega*Dt_AttCtl, -DELTA_THRUST_MAX_SETPOINT, DELTA_THRUST_MAX_SETPOINT);
	Torques[PITCH] = Saturate(ErrorOmega + Int_OmegaLoop[PITCH], -DELTA_THRUST_MAX_SETPOINT, DELTA_THRUST_MAX_SETPOINT);
	// YAW
	ErrorOmega = Kp_Omega[YAW]*(OmegaSetPoint[YAW] - Omega_hat[YAW]);
	Int_OmegaLoop[YAW] = Saturate( Int_OmegaLoop[YAW] + Ki_Omega[YAW]*ErrorOmega*Dt_AttCtl, -DELTA_THRUST_MAX_SETPOINT_YAW, DELTA_THRUST_MAX_SETPOINT_YAW);
	Torques[YAW] = Saturate(ErrorOmega + Int_OmegaLoop[YAW], -DELTA_THRUST_MAX_SETPOINT_YAW, DELTA_THRUST_MAX_SETPOINT_YAW);

	// Determine the corresponding rotor's speed for previous torques
	Command[0] = ThrottleSetPoint/(cos(EulerAngles[ROLL])*cos(EulerAngles[PITCH]));
	Command[1] = Torques[ROLL];
	Command[2] = Torques[PITCH];
	Command[3] = Torques[YAW];

	for (i=0; i<4; i++)
	{
		Omega_Rotor[i] = 0.0;
		for (j=0; j<4; j++)
		{
			Omega_Rotor[i] += iGamma[i][j]*Command[j];
		}
		Omega_Rotor[i] = sqrt(Saturate(Omega_Rotor[i], OMEGA_ROTOR_SQUARE_MIN, OMEGA_ROTOR_SQUARE_MAX));
		// convert the rotor speed to PWM
		motor[i] = Saturate(a_PWM*Omega_Rotor[i] + b_PWM, PWM_MIN, PWM_MAX);
	}
}

float DeadZone(float Input, float DeadZone)
{
	if ( (Input < DeadZone) && (Input > -DeadZone) )
	{
		return(0.0);
	}
	else
	{
		if (Input >0)
		{
			return(Input - DeadZone);
		}
		else
		{
			return(Input + DeadZone);
		}
	}
}

float Saturate(float Input, float Min, float Max)
{
	if(Input < Min)
	{
		return(Min);
	}
	else if (Input > Max)
	{
		return(Max);
	}
	else
	{
		return(Input);
	}
}
