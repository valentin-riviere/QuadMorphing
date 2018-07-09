%
% PRECAUTIONS:
% Before run this script, be sure to have correctly complete the TWO models
% QUARC_your_model_name and VERDEX_your_model_name.
%                                                                         %
% Augustin MANECY 29/05/2012                                              %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


clc;
clear all;
close all;

        % add to path TOOLBOX_GUMSTIX
%     display('Add GUMSTIX_VERDEX library to Matlab Path...')
%     addpath(pwd); % to know auto_path() function 
    
% %%%%%%%%%%%%%%%%%% section to be completed by user %%%%%%%%%%%%%%%%%%%%%

HostModelName = 'ViconController_x4morf_quat_withRecov_posMod';
ComModelName = 'ViconController_x4morf_quat_withRecov_posMod';
        
%%%%%%%%% PARAMETERES %%%%%%%%%%

% Sample time initialisation
ComMinSampleTime    = 1/400;     % put here the minimal sample time in seconds for gumstix (which will correspond to fundamental sample time)
HostMinSampleTime   = 1/400;     % put here your sample time in seconds for Host

Ts_AttitudeLoop = 1/200;
Ts_PositionLoop = 1/100;
Ts_Wifi = 1/200;
Ts_RCB = 1/400;
Ts_DSM = 1/50;      
Ts_Display = 1/20;

% specifics initialisation for model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    LandingHeight = 0.08;   % [m] height under which the inegrator (position loop) and the thrust feedforward are stopped
    g = 9.81;
    Mag_ref = [23806.2;...
               395.3;...
               39685.3];
    
    Ts_Att_EST = ComMinSampleTime;
    
    MPU6050.ADC.res = 16;                               % 16 bit ADC
    MPU6050.gyr.res_deg = 2000;                         % +- 2000deg
    MPU6050.gyr.res_rad = MPU6050.gyr.res_deg*pi/180;	% +- 2000deg
    MPU6050.acc.res = 8*g;                              % +- 8g
    MPU6050.gyr.int16_2_rad = MPU6050.gyr.res_rad/(2^(MPU6050.ADC.res-1));
    MPU6050.acc.int16_2_g = MPU6050.acc.res/(2^(MPU6050.ADC.res-1));
    
    gyrX_bias = 0; % [deg]
    gyrY_bias = 0; % [deg]
    gyrZ_bias = 0; % [deg]
    
    GyroX_bias0 = pi/180*gyrX_bias;
    GyroY_bias0 = pi/180*gyrY_bias;
    GyroZ_bias0 = pi/180*gyrZ_bias;
    
    AccBias = [0;0;0];
    
    phi_est0 = 0;
    theta_est0 = 0;
    psi_est0 = 0;

    b_est0 = [GyroX_bias0;...
              GyroY_bias0;...
              GyroZ_bias0];
    R_est0 = Euler2RotMat2(phi_est0, theta_est0, psi_est0, 'XYZ');
    
    b_est0 = single(b_est0);
    R_est0 = single(R_est0);
    
    display('     done!');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Quadri's parameters
RobotParameters;

% Acc calibration
load('AccCalib_Transformation_X4morf_Robot');

% Model for folding/unfolding
load('modelFolding3');

%%
% Generation of files to copy on the gumstix
GenereCOM(ComModelName, ComMinSampleTime);

%% DO NOT MODIFY THIS PART!
CleanUpHostModel(HostModelName);
mex SfunCPP_RecoverVicon_SDK.cpp 
    % clean up all previous compiled file and directories
GenereHost(HostModelName, HostMinSampleTime);