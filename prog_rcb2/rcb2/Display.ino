/*
This file is part of QuadMorphing project
Copyright (C) 2018 - Institute of Movement Sciences (Marseille, FRANCE)

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
extern int32_t t_exec_oneStep;
extern int32_t t_exec_Pilot_loop;
extern int32_t t_exec_RX_loop;
extern int32_t t_exec_CheckBattery;
extern int32_t t_exec_DisplayMenu;
extern int32_t t_exec_max_oneStep;
extern int32_t t_exec_max_Pilot_loop;
extern int32_t t_exec_max_RX_loop;
extern int32_t t_exec_max_CheckBattery;
extern int32_t t_exec_max_DisplayMenu;
extern float accelBias[3];

void DisplayMenu(void)
{
	char str[100];
	
	sprintf((char*)str,"\nRotor\t\t1\t2\t3\t4\n--Tachys raw--\t%d\t%d\t%d\t%d",TachyRisingPeriod[0],TachyRisingPeriod[1],TachyRisingPeriod[2],TachyRisingPeriod[3]);
	Serial.println(str);
	sprintf((char*)str,"--Tachys freq--\t%.0f\t%.0f\t%.0f\t%.0f",dispRotorSpeed[0],dispRotorSpeed[1],dispRotorSpeed[2],dispRotorSpeed[3]);
	Serial.println(str);
	sprintf((char*)str,"--Motors cons--\t%d\t%d\t%d\t%d",motor[0],motor[1],motor[2],motor[3]);
	Serial.println(str);
	sprintf((char*)str,"--SetPoints--\t%d\t%d\t%d\t%d",SetPoint_us[0],SetPoint_us[1],SetPoint_us[2],SetPoint_us[3]);
	Serial.println(str);
	sprintf((char*)str,"--TimeUpPWM--\t%.2f\t%.2f\t%.2f\t%.2f",RegulSig.TimeUp_PWM_ESC[0],RegulSig.TimeUp_PWM_ESC[1],RegulSig.TimeUp_PWM_ESC[2],RegulSig.TimeUp_PWM_ESC[3]);
	Serial.println(str);
	sprintf((char*)str,"\nRAW IMU\t\tROLL\tPITCH\tYAW\n--Acc--\t\t%f\t%f\t%f\tg",accADC[ROLL],accADC[PITCH],accADC[YAW]);
	Serial.println(str);
	sprintf((char*)str,"\nUNBIAS IMU\t\tROLL\tPITCH\tYAW\n--Acc--\t\t%f\t%f\t%f\tg",accADC[ROLL]+accelBias[0],accADC[PITCH]+accelBias[1],accADC[YAW]-accelBias[2]);
	Serial.println(str);
	sprintf((char*)str,"--Gyro--\t%f\t%f\t%f\trad/s",gyroADC[ROLL],gyroADC[PITCH],gyroADC[YAW]);
	Serial.println(str);
	sprintf((char*)str,"--Mag--\t\t%f\t%f\t%f\tmG",magADC[ROLL],magADC[PITCH],magADC[YAW]);
	Serial.println(str);
	sprintf((char*)str,"--North--\t\t%f",atan2(magADC[1],magADC[0])*rad2deg);
	Serial.println(str);
	sprintf((char*)str,"--Bat--\tRaw:%d\tin Volts: %fV",vbatRaw,(float)vbatRaw*V_PER_BIT*VBAT_SCALEFACTOR);
	Serial.println(str);
	sprintf((char*)str,"Euler Angles:\n%.1f\t%.1f\t%.1f",EulerAngles[0]*rad2deg,EulerAngles[1]*rad2deg,EulerAngles[2]*rad2deg);
	Serial.println(str);
	
	Display_RX();
}

void Display_RX(void)
{
	char str[100];
	
	Serial.println("THROTTLE\tROLL\tPITCH\tYAW\tMANU\tAUX2\tARMED?");
	sprintf((char*)str,"%d\t\t%d\t%d\t%d\t%d\t%d\t%d",rcData[THROTTLE],rcData[ROLL],rcData[PITCH],rcData[YAW],rcData[AUX1],rcData[AUX2],MotorsArmed);
	Serial.println(str);
}

void DisplayTimeExecRCB2(void)
{
	Serial.print("oneStep(): ");
	Serial.println(t_exec_oneStep);
	Serial.print("oneStep() Max: ");
	Serial.println(t_exec_max_oneStep);
	
	Serial.print("pilot_loop(): ");
	Serial.println(t_exec_Pilot_loop);
	Serial.print("pilot_loop() Max: ");
	Serial.println(t_exec_max_Pilot_loop);

	Serial.print("RX_loop(): ");
	Serial.println(t_exec_RX_loop);
	Serial.print("RX_loop() Max: ");
	Serial.println(t_exec_max_RX_loop);
	
	Serial.print("CheckBattery(): ");
	Serial.println(t_exec_CheckBattery);
	Serial.print("CheckBattery() Max: ");
	Serial.println(t_exec_max_CheckBattery);
	
	Serial.print("DisplayMenu(): ");
	Serial.println(t_exec_DisplayMenu);
	Serial.print("DisplayMenu() Max: ");
	Serial.println(t_exec_max_DisplayMenu);
	
	Serial.print("All: ");
	Serial.println(t_exec_oneStep+t_exec_Pilot_loop+t_exec_RX_loop+t_exec_CheckBattery+t_exec_DisplayMenu);
}