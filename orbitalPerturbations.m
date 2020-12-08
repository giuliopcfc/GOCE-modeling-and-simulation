function [aDrag, aJ2] = orbitalPerturbations(rr,vv,data)
% 
% Function to compute the drag and J2 perturbing accelerations.
% 
% INPUT:
%  rr   [3,1]     Position Vector
%  vv   [3,1]     Velocity Vector
%  data           data struct.
% 
% OUTPUT:
%  aDrag [3,1]    Perturbing acceleration vector due to aerodynamic drag [m/s^2]
%  aJ2   [3,1]    Perturbing acceleration vector due to J2 [km/s^2]
% 

% Drag perturbing acceleration:
rho = density(rr,data);

vRel = (vv - cross(data.const.W_EARTH*[0;0;1],rr))*1000; % Air Relative Velocity [m/s]

aDrag = -0.5/data.goce.balCoeff*rho*norm(vRel)*vRel;     % Perturbing acceleration [m/s^2]

% J2 perturbing acceleration:
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
