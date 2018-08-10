function cellInfo = UAVstruct_BusObject(varargin) 
% UAVSTRUCT_BUSOBJECT returns a cell array containing bus object information 
% 
% Optional Input: 'false' will suppress a call to Simulink.Bus.cellToObject 
%                 when the MATLAB file is executed. 
% The order of bus element attributes is as follows:
%   ElementName, Dimensions, DataType, SampleTime, Complexity, SamplingMode, DimensionsMode, Min, Max 

suppressObject = false; 
if nargin == 1 && islogical(varargin{1}) && varargin{1} == false 
    suppressObject = true; 
elseif nargin > 1 
    error('Invalid input argument(s) encountered'); 
end 

cellInfo = { ... 
  { ... 
    'estimation', ... 
    '', ... 
    sprintf(''), ... 
    'Auto', ... 
    '-1', {... 
      {'k_a', [1 3], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'k_m', [1 3], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'k_b', [1 3], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'gyr_bias_slewRate', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
    } ...
  } ...
  { ... 
    'constant', ... 
    '', ... 
    sprintf(''), ... 
    'Auto', ... 
    '-1', {... 
      {'g', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'mag_ref', [3 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
    } ...
  } ...
  { ... 
    'acc', ... 
    '', ... 
    sprintf(''), ... 
    'Auto', ... 
    '-1', {... 
      {'bias', [3 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'noise', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'noise_on', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
    } ...
  } ...
  { ... 
    'gyr', ... 
    '', ... 
    sprintf(''), ... 
    'Auto', ... 
    '-1', {... 
      {'bias', [3 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'noise', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'noise_on', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
    } ...
  } ...
  { ... 
    'mag', ... 
    '', ... 
    sprintf(''), ... 
    'Auto', ... 
    '-1', {... 
      {'bias', [3 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'noise', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'noise_on', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
    } ...
  } ...
  { ... 
    'sensors', ... 
    '', ... 
    sprintf(''), ... 
    'Auto', ... 
    '-1', {... 
      {'mag', 1, 'mag', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'acc', 1, 'acc', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'gyr', 1, 'gyr', -1, 'real', 'Sample', 'Fixed', [], []}; ...
    } ...
  } ...
  { ... 
    'control', ... 
    '', ... 
    sprintf(''), ... 
    'Auto', ... 
    '-1', {... 
      {'kR', [3 3], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'kO', [3 3], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'Delta_Thrust_sat', [3 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'SatThrust', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'SatPhi', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'SatTheta', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
    } ...
  } ...
  { ... 
    'UavStruct', ... 
    '', ... 
    sprintf(''), ... 
    'Auto', ... 
    '-1', {... 
      {'constant', 1, 'constant', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'m', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'Kv', [3 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'Kr', [3 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'nb_rotor', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'D', [4 3], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'I', [3 3], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'Ir', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'theta0', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'theta1', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'a', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'A1s', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'A1c', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'r', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'cT', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'cQ', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'sigma', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'gamma', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'Tmin', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'Tmax', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'omega_r_min', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'omega_r_max', [1 1], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'rotor_direction', [1 4], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'sensors', 1, 'sensors', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'estimation', 1, 'estimation', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'control', 1, 'control', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'TorquesSat', [3 90], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
      {'iGamma', [4 4 90], 'double', -1, 'real', 'Sample', 'Fixed', [], []}; ...
    } ...
  } ...
}'; 

if ~suppressObject 
    % Create bus objects in the MATLAB base workspace 
    Simulink.Bus.cellToObject(cellInfo) 
end 