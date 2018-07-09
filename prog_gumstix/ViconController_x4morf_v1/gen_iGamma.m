function [ iG , TorquesSat ] = gen_iGamma(l_b,L,cT_unfold,cT_fold,cQ,omega_r_min,omega_r_max)
% Inverse Gamma matrix
% OUTPUTS
%	- iG			:	Inverted matrix for each angle gamma 0 to 89 deg		[4x4]
%	- TorquesSat	:	Torques saturation for each angle gamma 0 to 89 deg		[4x4]

iG = zeros(4,4,181);
TorquesSat = zeros(3,181);

for gamma = 0:1:180

    cT = cT_unfold;
    cT2 = (cT_fold-cT_unfold)/89*gamma+cT_unfold;
	    
	Gamma = [   cT                        ,   cT2                          ,  cT2                           , cT			;...
				-cT*l_b/2*cosd(gamma)      ,   cT2*l_b/2*cosd(gamma)        ,  cT2*l_b/2*cosd(gamma)         , -cT*l_b/2*cosd(gamma)	;...
				-cT*(L/2-l_b/2*sind(gamma)),   -cT2*(L/2+l_b/2*sind(gamma)) ,  cT2*(L/2+l_b/2*sind(gamma))	, cT*(L/2-l_b/2*sind(gamma))		;...
				cQ                          ,   -cQ                         ,  cQ                           , -cQ			];

	iG(:,:,gamma+1) = pinv(Gamma);
	TorquesSat(1,gamma+1) = abs(Gamma(2, :)*[omega_r_min^2;...
                                 omega_r_max^2; 
                                 omega_r_max^2; 
                                 omega_r_min^2]);
	TorquesSat(2,gamma+1) = abs(Gamma(3, :)*[omega_r_min^2;...
                                 omega_r_min^2; 
                                 omega_r_max^2; 
                                 omega_r_max^2]);
	TorquesSat(3,gamma+1) = abs(Gamma(4, :)*[omega_r_max^2;...
                                 omega_r_min^2; 
                                 omega_r_max^2; 
                                 omega_r_min^2]);
end
		
end