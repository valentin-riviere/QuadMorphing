% This file is part of QuadMorphing project
% Copyright (C) 2018 - Institute of Movement Sciences (Marseille, FRANCE)

% This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

% This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
% See the GNU General Public License for more details.

% You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.

clear all;
clc;

%% PARAMETERS: 
N = 200;    % Number of measure per orientation

%% Loading data
load data_acc.mat;	% Name of the variables (.mat) where to find measures

% Loading measures
rec = data(14,:);
i_b = find([0 diff(rec)] > 0); K = length(i_b);
mes = cell(1,K); j = 1;
for i = i_b
	mes{j} = data(2:13,1:N*pas);
    j = j+1;
end

% R0 Matrix (horizontal position)
R0 = reshape(mean(mes{1}(4:12,:),2),[3 3]);

% Gr and A Matrix
A = -ones(N*K,4);
Gr = zeros(N*K,3);
for j=1:K
    for i = 1:N
        A((j-1)*N+i,1:3) = mes{j}(1:3,i);
        Gr((j-1)*N+i,1:3) = [0 0 -9.81]*R0'*reshape(mes{j}(4:12,i),[3 3]);
    end
end

% C Matrix
C = A\Gr;

% Bias and scale
T = C(1:3,1:3)'
b = T\(C(4,1:3)')