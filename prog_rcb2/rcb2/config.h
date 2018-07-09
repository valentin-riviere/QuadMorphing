/*
This file is part of QuadMorphing project
Copyright (C) 2018 - Institute of Movement Sciences (Marseille, FRANCE)

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/* MPU9250 Low pass filter setting. In case you cannot eliminate all vibrations to the Gyro, you can try
to decrease the LPF frequency, only one step per try. As soon as twitching gone, stick with that setting.
It will not help on feedback wobbles, so change only when copter is randomly twiching and all dampening and
balancing options ran out. Uncomment only one option!
IMPORTANT! Change low pass filter setting changes PID behaviour, so retune your PID's after changing LPF.*/
 
#ifndef CONFIG_H
#define CONFIG_H

//#define MPU9250_LPF_256HZ     // This is the default setting, no need to uncomment, just for reference
//#define MPU9250_LPF_188HZ
//#define MPU9250_LPF_98HZ
//#define MPU9250_LPF_42HZ
#define MPU9250_LPF_20HZ
//#define MPU9250_LPF_10HZ
//#define MPU9250_LPF_5HZ       // Use this only in extreme cases, rather change motors and/or props

//MPU9250 Gyro LPF setting
#if defined(MPU9250_LPF_256HZ) || defined(MPU9250_LPF_188HZ) || defined(MPU9250_LPF_98HZ) || defined(MPU9250_LPF_42HZ) || defined(MPU9250_LPF_20HZ) || defined(MPU9250_LPF_10HZ) || defined(MPU9250_LPF_5HZ)
  #if defined(MPU9250_LPF_256HZ)
    #define MPU9250_DLPF_CFG   0
  #endif
  #if defined(MPU9250_LPF_188HZ)
    #define MPU9250_DLPF_CFG   1
  #endif
  #if defined(MPU9250_LPF_98HZ)
    #define MPU9250_DLPF_CFG   2
  #endif
  #if defined(MPU9250_LPF_42HZ)
    #define MPU9250_DLPF_CFG   3
  #endif
  #if defined(MPU9250_LPF_20HZ)
    #define MPU9250_DLPF_CFG   4
  #endif
  #if defined(MPU9250_LPF_10HZ)
    #define MPU9250_DLPF_CFG   5
  #endif
  #if defined(MPU9250_LPF_5HZ)
    #define MPU9250_DLPF_CFG   6
  #endif
#else
    //Default settings LPF 256Hz/8000Hz sample
    #define MPU9250_DLPF_CFG   0
#endif

#endif