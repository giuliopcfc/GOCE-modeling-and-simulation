function [aThrust] = ionThruster(xV,data)
% 
% Function to compute the thrust.
% 
% INPUT:
%  xV                 Position of the spool [m]
%  data               data struct
% 
% OUTPUT:
%  aThrust            Thrust acceleration [m/s^2]          
% 

D0 = data.FCV.D0;

% BCS on xFCV:
if xV > D0
    xV = D0;
elseif xV < 0
    xV = 0;
end

% assigning the value to z
z = 1-xV/D0;

% computation of the orifice area
alpha = 2*acos(1-2*z);
areaOrifice = D0^2/8*(alpha - sin(alpha));

% since the pressure after the valve is much lower than the critical
% pressure, the flow is chocked. Hypothesis of homoentropic flow.
T2 = data.thruster.T2;
p2 = data.thruster.p2;
k =  data.thruster.kXe; 
R = data.const.Ru/data.thruster.mmXe;

mDot = areaOrifice*p2*sqrt(k)/sqrt(T2*R)*(1+0.5*(k-1))^((-k-1)/2/(k-1));

vAcc = sqrt(2*data.thruster.e*data.thruster.deltaV/data.thruster.massIon);

aThrust = mDot*vAcc./data.goce.mass;

end