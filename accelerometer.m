function [dYA, VC] = accelerometer(YA, thrust, dragV, data)
% 
% Function to compute the derivative of the state of a capacitive
% accelerometer + read-out circuit for voltage.
%  
% INPUT:
%  YA     [3,1]     Array of the state variables (position, velocity and
%                   output voltage of the read-out circuit)
%  thrust           Value of thrust [N]
%  drag             Value of drag component along velocity direction [N]
%  data             data struct
% 
% OUTPUT:
%  dYA    [3,1]     Time derivative of the state vector
 
% Load data:
mGOCE = data.goce.mass;
areaA = data.accelerometer.areaMass;
massA = data.accelerometer.mass;

perm = data.accelerometer.eps;
g = data.accelerometer.gap;
VBias = data.accelerometer.VBias;
CF = data.accelerometer.C;

kProp = data.accelerometer.kProp;
kDer = data.accelerometer.kDer;

% Load state variables:
x = YA(1); v = YA(2); VOut = YA(3);

% BCS for the accelerometer (physical constraints):
if x <= -g
    x = -g;
    if v <= 0
        v = 0;
    end
end
if x >= +g
    x = g;
    if v >= 0
        v = 0;
    end
end

% Variable capacitances of the accelerometer:
C1 = perm*areaA/(g - x);
C2 = perm*areaA/(g + x);

% Voltage drop on the seismic mass:
Vx = x/g*VBias;

% Acceleration of the seismic mass due to the acceleration of the s/c:
a_ext = (thrust + dragV)/mGOCE; 

% Current flowing from the seismic mass:
dC = perm*areaA*(1/(g + x)^2 + 1/(g - x)^2);
i = dC*v*VBias;

% Derivative of the output voltage:
dVOut = -i/CF;

% Control voltage (PD):
VC = kProp*VOut + kDer*dVOut;

% Electrostatic forces acting on the mass:
DV1 = VBias - VC - 1/2*Vx;
DV2 = VBias + VC + 1/2*Vx;
F1 = 0.5*C1*DV1^2/(g - x);
F2 = 0.5*C2*DV2^2/(g + x);

% EOM of the seismic mass:
dx = v;
dv = a_ext + (-F1 + F2)/massA;

% BCS for the accelerometer acceleration:
if (x >= g && dv > 0) || (x <= -g && dv < 0)
    dv = 0;
end

% Derivatives of the state variables:
dYA = [dx; dv; dVOut];

end