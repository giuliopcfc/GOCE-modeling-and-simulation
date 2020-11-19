function aJ2 = pertJ2(rr,data)
% 
% Function to compute the perturbing acceleration due to J2 effect.
%
% PROTOTYPE:
%  aJ2 = pertJ2(rr,k_E)
%  
% INPUT:
%  rr   [3,1]    Position vector in inertial frame [km]
%  data          data struct
% 
% OUTPUT:
%  aJ2 [3,1]     Perturbing acceleration vector due to J2 [km/s^2]
% 

J2 = data.const.J2; 
R_EQ = data.const.R_EQUATORIAL; 
MU = data.const.MU_EARTH;

X = rr(1);
Y = rr(2);
Z = rr(3);
r = norm(rr);

aJ2 = 3/2*J2*MU*R_EQ^2/r^4*[X/r*(5*Z^2/r^2-1);... 
                            Y/r*(5*Z^2/r^2-1);...
                            Z/r*(5*Z^2/r^2-3)];

end