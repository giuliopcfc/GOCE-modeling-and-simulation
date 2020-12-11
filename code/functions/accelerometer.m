function [dYA] = accelerometer(YA, aThrust, aDragV, data)
 % 
 % Function to compute the derivative of the state of a capacitive
 % accelerometer + read-out circuit for voltage.
 % 
 % INPUT:
 % YA     [3,1]     Array of the state variables (position, velocity and
 %                  output voltage of the read-out circuit)
 % aThrust          Thrust acceleration [m/s^2]
 % aDragV           Drag acceleration in velocity direction [m/s^2]
 % data             data struct
 % 
 % OUTPUT:
 % dYA    [3,1]     Time derivative of the state vector
 
% Load data:
areaA = data.accelerometer.areaMass;
massA = data.accelerometer.mass;

perm = data.accelerometer.eps;
g = data.accelerometer.gap;
VBias = data.accelerometer.VBias;
CF = data.accelerometer.C;

kProp = data.accelerometer.kProp;
kDer = data.accelerometer.kDer;

% Load state variables:
xA = YA(1); vA = YA(2); VOut = YA(3);

% BCS for the accelerometer (physical constraints):
if xA <= -g
    xA = -g;
    if vA <= 0
        vA = 0;
    end
end
if xA >= +g
    xA = g;
    if vA >= 0
        vA = 0;
    end
end

% Variable capacitances of the accelerometer:
C1 = perm*areaA/(g - xA);
C2 = perm*areaA/(g + xA);

% Voltage drop on the seismic mass:
Vx = xA/g*VBias;

% Acceleration of the seismic mass due to the acceleration of the s/c:
a_ext = aThrust + aDragV;

% Current flowing from the seismic mass:
dC = perm*areaA*(1/(g + xA)^2 + 1/(g - xA)^2);
iF = dC*vA*VBias;

% Derivative of the output voltage:
dVOut = -iF/CF;

% Control voltage (PD):
VC = kProp*VOut + kDer*dVOut;

% Electrostatic forces acting on the mass:
DV1 = VBias - VC - 1/2*Vx;
DV2 = VBias + VC + 1/2*Vx;
F1 = 0.5*C1*DV1^2/(g - xA);
F2 = 0.5*C2*DV2^2/(g + xA);

% EOM of the seismic mass:
dxA = vA;
dvA = a_ext + (-F1 + F2)/massA;

% BCS for the accelerometer acceleration:
if (xA >= g && dvA > 0) || (xA <= -g && dvA < 0)
    dvA = 0;
end

% Derivatives of the state variables:
dYA = [dxA; dvA; dVOut];

end