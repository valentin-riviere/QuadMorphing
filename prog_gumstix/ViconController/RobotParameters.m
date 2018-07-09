% universal constants
g = 9.81;           % [m.s^-2] gravition constant
rho_air = 1.2041;   % [kg.m^-3] air density
mag_ref = [23806.2;...
           395.3;...
           39685.3]; % [nT] reference magnetic field (marseille)

deg2rad  = pi/180;
rad2deg = 180/pi;

%% UAV Parameters

% Folding parameters
time_unfolding = 230e-3;

% mass and dimension for default position
m = 0.380;		% [kg]
Ix = 8e-4;	% [kg.m^2]  UAV inertia momentum around X-axis in body frame
Iy = 3.23e-3;	% [kg.m^2]  UAV inertia momentum around X-axis in body frame
Iz = 3.2e-3;	% [kg.m^2]  UAV inertia momentum around X-axis in body frame

L = 0.28;       % [m]   length of quadrotor
l_b = 0.14;     % [m]   width of quadrotor
hG = 0;         % [m]   height relative to body frame's origin of the center of mass
hT = 55e-3;     % [m]   height of the thrust center of each rotor relative to center of mass

D = [L/2 , -l_b/2 , hT;...
     L/2 , l_b/2 , hT;...
     -L/2, l_b/2 , hT;...
     -L/2 , -l_b/2, hT];

I = [Ix, 0 , 0 ;...
      0, Iy, 0 ;...
      0, 0 , Iz];

% translational aerodynamic effects
Kvx = 0.05;     % [N.s.m^-1]    drag coefficient for translation on X-axis
Kvy = 0.05;     % [N.s.m^-1]    drag coefficient for translation on Y-axis
Kvz = 0.05;     % [N.s.m^-1]    drag coefficient for translation on Z-axis

Kv = [Kvx, Kvy, Kvz]';

% rotational aerodynamic effects (don't used yet)
Krx = 0.05;     % [N.m.s^2.rad^-2]  rotational frictions aerodynamics coefficients around X-axis in body frame
Kry = 0.05;     % [N.m.s^2.rad^-2]  rotational frictions aerodynamics coefficients around Y-axis in body frame
Krz = 0.05;     % [N.m.s^2.rad^-2]  rotational frictions aerodynamics coefficients around Z-axis in body frame

Kr = [Krx, Kry, Krz]';

% propellers characteristics
nb_rotor = 4;       % [-] number of rotors
rotor_direction = [-1 1 -1 1];  % sense of rotation of each rotor
Ib = 1.345e-6;      % [kg.m^2] blade inertia momentum
Im = 2.866e-7/2;    % [kg.m^2] motor inertia momentum
Ir = Ib+Im;         % [kg.m^2] complete rotor inertia momentum
theta0 = 24;        % [deg] angle of attack at the rotor shaft
theta1 = -10;       % [deg] equivalent twist angle (due to chord reduction)
c = 1.6e-2;         % [m] rotor chord
r = 6.0e-2;         % [m] rotor radius
a = 6.0;            % [-] lift slope coefficient

MinArmed = 1000;        % [us]
MinThrottle = 1200;     % [us]
MaxThrottle = 2000;     % [us]

Tprop_CL = 0.1; % [s] time constant of the closed loop for propeller's rotation rate (2nd order) tf(1, [1/(7*2*pi) 1])^2
cTm = 1.0194e-7;      % [kg.s^2.rad^-2] thrust coefficient
cQm = 1.0031e-9;     % [kg.s^2.rad^-2] drag coefficient
Tmax = .8155;       % [kg] maximum thrust achievable per rotor
Tmin = .0097;       % [kg] minimum thrust achievable per rotor (to ensure that omega_r > 0 and ensure the existence of flapping angle) 

%% deduction of other parameters
eff = 0.90;
cT = eff*cTm*g;                     % [N.s^2.rad^-2] thrust coefficient
% cT = 2.45e-6;
% cT_fold = 0.97e-6;
cT_fold = cT;
cQ = cQm*g;             % [-] drag coefficient
% cQ = 2.2e-8;
sigma = nb_rotor*c/(pi*r);      % [-] solidity ratio (Air of blade/ Air of rotor disc)
gamma = rho_air*a*c*r^4/Ib;     % [-] lock Number
theta0 = theta0*pi/180;         % [rad]
theta1 = theta1*pi/180;         % [rad]  
omega_r_min = sqrt(Tmin*g/cT);    % [rad.s^-1] minimum rotation rate of each rotor
omega_r_max = sqrt(Tmax*g/cT);    % [rad.s^-1] maximum rotation rate of each rotor

A1s = (4/3+8/9*cT*gamma/(sigma*a));
A1c = (8/3*theta0+2*theta1);

% control mixer matrix

% matrix for x configuration
Delta_Thrust_sat = [l_b*(cT*omega_r_max^2-cT*omega_r_min^2);...
					L*(cT*omega_r_max^2-cT*omega_r_min^2);...
					2*(cQ*omega_r_max^2-cQ*omega_r_min^2)];

Gamma = [   cT ,   cT ,   cT ,  cT  ;...
		  -l_b/2*cT,  l_b/2*cT,  l_b/2*cT, -l_b/2*cT;...
		  -L/2*cT, -L/2*cT,  L/2*cT, L/2*cT ;...
		  +cQ  ,  -cQ ,  +cQ , -cQ  ];
 
% control mixer matrix & torques saturation
[iGamma,TorquesSat] = gen_iGamma(l_b,L,cT,cT_fold,cQ,omega_r_min,omega_r_max);

%%%%%%%%%%%%%%%%%%%%%%%%%%%% CUSTOMIZATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

phi_ref = 0;    % [deg]
theta_ref = 0;  % [deg]
psi_ref = 0;    % [deg]

% saturation for position controller
phi_max    = 30;                % [deg]
theta_max  = 30;                % [deg]
Thrust_max = 90;                % [%] percent of maximal thrust asked for each motor...
AnglesMax = [phi_max; theta_max; inf];
AnglesMin = [-phi_max; -theta_max; -inf];
% OmegaMax = [800; 800; 500];     % [deg.s^-1]
% OmegaMin = -[800; 800; 500];    % [deg.s^-1]
%%%%%%%%%%%%%%%%%%%%%%%%%TEST%%%%%%%%%%%%%%%%%%%%%%%%%%
OmegaMax = [100; 100; 40];     % [deg.s^-1]
OmegaMin = -[100; 100; 40];    % [deg.s^-1]
dOmegaMax = I\TorquesSat;
dOmegaMin = -I\TorquesSat;
AnglesMax = deg2rad*AnglesMax;
AnglesMin = deg2rad*AnglesMin;
OmegaMax = deg2rad*OmegaMax;
OmegaMin = deg2rad*OmegaMin;

PositionMax = [5.7 ; 2.5; 3];             % [m]
PositionMin = [-0.2; -2.5; 0];             % [m]
PositionSecuMax = [5.5 ; 3 ; 4];        % [m]
PositionSecuMin = [-0.5; -3; -0.07];    % [m]
SpeedMax = [1; 1; 1];                   % [m.s^-1]
SpeedMin = [-1; -1; -1];                % [m.s^-1]
SpeedMaxScenario = [3; 1; 3];           % [m.s^-1]
SpeedMinScenario = [-3; -1; -3];        % [m.s^-1]
AccMax = [g*tan(AnglesMax(1)); g*tan(AnglesMax(2)); g*nb_rotor*Thrust_max/100*Tmax];
AccMin = -[g*tan(AnglesMax(1)); g*tan(AnglesMax(2)); g*nb_rotor*Tmin];
AccMaxScenario = [g*tan(AnglesMax(1)); g*tan(AnglesMax(2)); g*nb_rotor*Thrust_max/100*Tmax];
AccMinScenario = -[g*tan(AnglesMax(1)); g*tan(AnglesMax(2)); g*nb_rotor*Tmin];

X_int_sat = [2, 2, 2]';	% [m] Maximum value of the integral of position's error
% X_int_sat = [0.5, 2, 2]';

% initial states of robot
Vx_0 = 0;
Vy_0 = 0;
Vz_0 = 0;
Velocity_0 = [Vx_0 Vy_0 Vz_0];

X_0 = 0;
Y_0 = 0;
Z_0 = 0;

p_0 = 0;
q_0 = 0;
r_0 = 0;

phi_0 = 0;
theta_0 = 0;
psi_0 = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%% END CUSTOMIZATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% PWM initialization

a_PWM = (MaxThrottle-MinThrottle)/(omega_r_max-omega_r_min);
b_PWM = MinThrottle - a_PWM*omega_r_min;  

%% sensors intialization

accX_bias = 0.2;
accY_bias = -0.05;
accZ_bias = 0.26;

gyrX_bias = 2.25;       % p, -> roll    [deg/s]
gyrY_bias = -0.58;      % q, -> pitch   [deg/s]
gyrZ_bias = -1.37;      % r, -> yaw     [deg/s]

%% ServoMotor for folding

gamma_max = 89; % in deg
dc_close = 78;    % in % corresponds to gamma_max
dc_open = 28;    % in % corresponds to gamma = 0°

%% UAV STRUCTURE

% structure definition

UAVstruct.constant.g = g;
UAVstruct.constant.mag_ref = mag_ref;
UAVstruct.m = m;
UAVstruct.Kv = Kv;
UAVstruct.Kr = Kr;
UAVstruct.nb_rotor = nb_rotor;
UAVstruct.D = D;
UAVstruct.I = I;
UAVstruct.Ir = Ir;
UAVstruct.theta0 = theta0;
UAVstruct.theta1 = theta1;
UAVstruct.a = a;
UAVstruct.A1s = A1s;
UAVstruct.A1c = A1c;
UAVstruct.r = r;
UAVstruct.cT = cT;
UAVstruct.cQ = cQ;
UAVstruct.sigma = sigma;
UAVstruct.gamma = gamma;
UAVstruct.Tmin = Tmin;
UAVstruct.Tmax = Tmax;
UAVstruct.omega_r_min = omega_r_min;
UAVstruct.omega_r_max = omega_r_max;
UAVstruct.rotor_direction = rotor_direction;

UAVstruct.sensors.acc.bias = [accX_bias;...
                              accY_bias;...
                              accZ_bias];

UAVstruct.sensors.gyr.bias = pi/180*[gyrX_bias;...
                                     gyrY_bias;...
                                     gyrZ_bias];

UAVstruct.control.Delta_Thrust_sat = Delta_Thrust_sat;

UAVstruct.TorquesSat = TorquesSat;
UAVstruct.iGamma = iGamma;

% generate the busobject corresponding to the structure
UAVstruct_BusObject;