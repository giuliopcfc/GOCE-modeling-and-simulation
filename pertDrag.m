function aDrag = pertDrag(rr,vv,data)
% 
% Function to compute the drag perturbing acceleration.
% 
% INPUT:
%  rr   [3,1]       Position Vector
%  vv   [3,1]       Velocity Vector
%  data             data struct.
% 
% OUTPUT:
%  aDrag [3,1]    Perturbing acceleration vector due to aerodynamic drag [m/s^2]
% 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SOSTITUIRE CON NUOVA FUNZIONE (COSPAR):
rho = density(rr,data);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

vRel = (vv - cross(data.const.W_EARTH*[0;0;1],rr))*1000; % Air Relative Velocity [m/s]

aDrag = -0.5/data.goce.balCoeff*rho*norm(vRel)*vRel; % Perturbing acceleration [m/s^2]
             
end
