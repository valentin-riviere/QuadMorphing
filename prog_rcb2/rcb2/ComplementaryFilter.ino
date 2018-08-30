/*
This file is part of QuadMorphing project
Copyright (C) 2018 - Institute of Movement Sciences (Marseille, FRANCE)

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "Def.h"

extern float deltat;
static float eInt[3] = {0.0f, 0.0f, 0.0f};

void EstimateAttitude(void)
{
#ifdef defined(CF_ESTIMATOR)
	EstimateAttitude_CF();
#elif defined(MAHONY_ESTIMATOR)
	EstimateAttitude_Mahony();
#else
	EstimateAttitude_Mahony();
#endif
}

void EstimateAttitude_CF(void)
{
	// around 1900us of execution time
	float a_bar[3];
	float a_hat[3];
	float a_tilde[3];
	float g_bar[3];
	float m_hat[3];
	float m_tilde[3];
	float Q_corr[4] = {0.0, 0.0, 0.0, 0.0};
	float alpha[3];
	float dB_hat[3];
	float dQ_hat[4];
	float norm_Q;
	int i;

	// accelerometer
	a_bar[0] = accADC[0];
	a_bar[1] = accADC[1];
	a_bar[2] = accADC[2];
	
	// magnetometer Not used by quat and euler angles
#ifdef MAG_USAGE
	m_bar[0] = magADC[0];
	m_bar[1] = magADC[1];
	m_bar[2] = magADC[2]; 
#else
	m_bar[0] = 1;
	m_bar[1] = 0;
	m_bar[2] = 0;
#endif

	// gyro
	g_bar[0] = gyroADC[0];
	g_bar[1] = gyroADC[1];
	g_bar[2] = gyroADC[2];

	// estimation of IMU vector using Q_hat (estimated quaternion): a_hat = Inv(Q_hat) * g
	/*
	  Inv(Q_hat) = [q0 -q1 -q2 -q3]'
	  Inv(Q_hat)*g = [1-2(q2^2+q3^2)	 2(q1q2+q0q3)	  2(q1q3-q0q2)]		[0]
					 [	2(q1q2-q0q3)   1-2(q1^2+q3^2))	  2(q2q3+q0q1)]	 *	[0]
					 [	2(q1q3+q0q2)	 2(q2q3-q0q1)	1-2(q1^2+q2^2)]		[g]
	*/
	a_hat[0] = -g_double*(Q_hat[1]*Q_hat[3] - Q_hat[0]*Q_hat[2]);
	a_hat[1] = -g_double*(Q_hat[2]*Q_hat[3] + Q_hat[0]*Q_hat[1]);
	a_hat[2] = -g_earth*(1-2.0*(Q_hat[1]*Q_hat[1] + Q_hat[2]*Q_hat[2]));
	// cross(a_hat, a_bar)
	a_tilde[0] = a_hat[1]*a_bar[2] - a_hat[2]*a_bar[1];
	a_tilde[1] = a_hat[2]*a_bar[0] - a_hat[0]*a_bar[2];
	a_tilde[2] = a_hat[0]*a_bar[1] - a_hat[1]*a_bar[0];

	// CORRECTION FROM FICTIOUS MAGNETOMETER (to avoid drift of yaw)
	// estimation of IMU vector using Q_hat (estimated quaternion): m_hat = Inv(Q_hat) * m_ref = Inv(Q_hat) * [1.0, 0.0, 0.0]'
	/*
	  Inv(Q_hat) = [q0 -q1 -q2 -q3]'
	  Inv(Q_hat)*g = [1-2(q2^2+q3^2)	 2(q1q2+q0q3)	  2(q1q3-q0q2)]		[1]
					 [	2(q1q2-q0q3)   1-2(q1^2+q3^2))	  2(q2q3+q0q1)]	 *	[0]
					 [	2(q1q3+q0q2)	 2(q2q3-q0q1)	1-2(q1^2+q2^2)]		[0]
	*/
	m_hat[0] = (1-2.0*(Q_hat[2]*Q_hat[2] + Q_hat[3]*Q_hat[3]));
	m_hat[1] = 2.0*(Q_hat[1]*Q_hat[2] - Q_hat[0]*Q_hat[3]);
	m_hat[2] = 2.0*(Q_hat[1]*Q_hat[3] + Q_hat[0]*Q_hat[2]);
	// compute the error between a_hat and a_tilde
	// cross(a_hat, a_bar)
	m_tilde[0] = m_hat[1]*m_bar[2] - m_hat[2]*m_bar[1];
	m_tilde[1] = m_hat[2]*m_bar[0] - m_hat[0]*m_bar[2];
	m_tilde[2] = m_hat[0]*m_bar[1] - m_hat[1]*m_bar[0];

	for (i=0; i<3; i++)
	{
		// Compute the debiased rotation speed
		Omega_hat[i] = g_bar[i] - B_hat[i];
		  
		// calculate the correction to apply to the quaternion
		alpha[i] = (k_a[i]*a_tilde[i])/g_squared + (k_m[i]*m_tilde[i]);

		// Corrected pure rotation quaternion for integration
		Q_corr[i+1] = Omega_hat[i] - alpha[i];

		// Bias derivative
		dB_hat[i] = k_b[i] * alpha[i];

		// Bias integration
		B_hat[i] += dB_hat[i]*Dt_CF;
	}

	// Quaternion derivative: dQ_hat = 0.5*(Q_hat*Q_corr)
	dQ_hat[0] = 0.5*(Q_hat[0]*Q_corr[0] - (Q_hat[1]*Q_corr[1] + Q_hat[2]*Q_corr[2] + Q_hat[3]*Q_corr[3]));
	dQ_hat[1] = 0.5*(Q_hat[0]*Q_corr[1] + Q_corr[0]*Q_hat[1] + Q_hat[2]*Q_corr[3] - Q_hat[3]*Q_corr[2]);
	dQ_hat[2] = 0.5*(Q_hat[0]*Q_corr[2] + Q_corr[0]*Q_hat[2] + Q_hat[3]*Q_corr[1] - Q_hat[1]*Q_corr[3]);
	dQ_hat[3] = 0.5*(Q_hat[0]*Q_corr[3] + Q_corr[0]*Q_hat[3] + Q_hat[1]*Q_corr[2] - Q_hat[2]*Q_corr[1]);

	for (i=0; i<4; i++)
	{
		// Quaternion integration
		Q_hat[i] += dQ_hat[i]*Dt_CF;
	}
	// Quaternion Normalization
	norm_Q = sqrt(Q_hat[0]*Q_hat[0] + Q_hat[1]*Q_hat[1] + Q_hat[2]*Q_hat[2] + Q_hat[3]*Q_hat[3]);
	Q_hat[0] /= norm_Q;
	Q_hat[1] /= norm_Q;
	Q_hat[2] /= norm_Q;
	Q_hat[3] /= norm_Q;

	// deduce Euler Angles
	EulerAngles[0] = -atan2(2*(Q_hat[2]*Q_hat[3]-Q_hat[0]*Q_hat[1]), Q_hat[0]*Q_hat[0] - Q_hat[1]*Q_hat[1] - Q_hat[2]*Q_hat[2] + Q_hat[3]*Q_hat[3]);
	EulerAngles[1] = asin(2*(Q_hat[1]*Q_hat[3]+Q_hat[0]*Q_hat[2]));
	EulerAngles[2] = -atan2(2*(Q_hat[1]*Q_hat[2]-Q_hat[0]*Q_hat[3]), Q_hat[0]*Q_hat[0] + Q_hat[1]*Q_hat[1] - Q_hat[2]*Q_hat[2] - Q_hat[3]*Q_hat[3]);
}

void EstimateAttitude_Mahony(void)
{
	float q1 = Q_hat[0], q2 = Q_hat[1], q3 = Q_hat[2], q4 = Q_hat[3];   // short name local variable for readability
	float ax, ay, az;
	ax = accADC[0]; ay = accADC[1]; az = accADC[2];
	float gx = gyroADC[0], gy = gyroADC[1], gz = gyroADC[2];
	float mx, my, mz;
#ifdef MAG_USAGE
	mx = magADC[0];
	my = magADC[1];
	mz = magADC[2]; 
#else
	mx = 1;
	my = 0;
	mz = 0;
#endif
	float norm;
	float hx, hy, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	float pa, pb, pc;

	// Auxiliary variables to avoid repeated arithmetic
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;   

	// Normalise accelerometer measurement
	norm = sqrtf(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f / norm;        // use reciprocal for division
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrtf(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f / norm;        // use reciprocal for division
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
	hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
	bx = sqrtf((hx * hx) + (hy * hy));
	bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

	// Estimated direction of gravity and magnetic field
	vx = 2.0f * (q2q4 - q1q3);
	vy = 2.0f * (q1q2 + q3q4);
	vz = q1q1 - q2q2 - q3q3 + q4q4;
	wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
	wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
	wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);  

	// Error is cross product between estimated direction and measured direction of gravity
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
	if (KI_MAH > 0.0f)
	{
		eInt[0] += ex;      // accumulate integral error
		eInt[1] += ey;
		eInt[2] += ez;
	}
	else
	{
		eInt[0] = 0.0f;     // prevent integral wind up
		eInt[1] = 0.0f;
		eInt[2] = 0.0f;
	}

	// Apply feedback terms
	gx = gx + KP_MAH * ex + KI_MAH * eInt[0];
	gy = gy + KP_MAH * ey + KI_MAH * eInt[1];
	gz = gz + KP_MAH * ez + KI_MAH * eInt[2];

	// Integrate rate of change of quaternion
	pa = q2;
	pb = q3;
	pc = q4;
	q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
	q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
	q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
	q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

	// Normalise quaternion
	norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	norm = 1.0f / norm;
	Q_hat[0] = q1 * norm;
	Q_hat[1] = q2 * norm;
	Q_hat[2] = q3 * norm;
	Q_hat[3] = q4 * norm;

	// deduce Euler Angles
	EulerAngles[0] = -atan2(2*(Q_hat[2]*Q_hat[3]-Q_hat[0]*Q_hat[1]), Q_hat[0]*Q_hat[0] - Q_hat[1]*Q_hat[1] - Q_hat[2]*Q_hat[2] + Q_hat[3]*Q_hat[3]);
	EulerAngles[1] = asin(2*(Q_hat[1]*Q_hat[3]+Q_hat[0]*Q_hat[2]));
	EulerAngles[2] = -atan2(2*(Q_hat[1]*Q_hat[2]-Q_hat[0]*Q_hat[3]), Q_hat[0]*Q_hat[0] + Q_hat[1]*Q_hat[1] - Q_hat[2]*Q_hat[2] - Q_hat[3]*Q_hat[3]);
}