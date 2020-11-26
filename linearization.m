% LINEARIZATION OF THE MATHEMATICAL MODEL

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %AGGIORNARE LE EQUATIONI SE CAMBIANO!!!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
config;
syms intVOut xFCV vFCV xA vA VOut dragV
Y = [intVOut xFCV vFCV xA vA VOut];

%% FCV:
% Load data:
kI = data.FCV.kI;
kProp = data.FCV.kProp; kInt = data.FCV.kInt;
m = data.FCV.massSpool; kSpring = data.FCV.kSpring; c = data.FCV.c;
A0 = data.FCV.A0;


D0 = sqrt(4*A0/pi); % Diameter of the orifice

% PI controller:
iFCV = kProp*VOut + kInt*intVOut;

x0 = D0; % Rest position of the spring [m]

% EOM of the spool:
dxFCV = vFCV;

dvFCV = 1/m*( kSpring*(x0 - xFCV) - kI*iFCV - c*vFCV );

% Derivatives of the state variables:
dYFCV = [VOut, dxFCV, dvFCV];

%% Ion Thruster:

% assigning the value to z
z = 1-xFCV/D0;

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

thrust = mDot*vAcc;

%% Accelerometer:

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

% Variable capacitances of the accelerometer:
C1 = perm*areaA/(g - xA);
C2 = perm*areaA/(g + xA);

% Voltage drop on the seismic mass:
Vx = xA/g*VBias;

% Acceleration of the seismic mass due to the acceleration of the s/c:
a_ext = (thrust + dragV)/mGOCE; 

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

%% Full system:

% Derivatives of the state variables:
dYA = [dxA, dvA, dVOut];

dY = [dYFCV, dYA];

J = jacobian(dY,Y);
A = zeros(6,6);

for i = 1:6
    for j = 1:6
        if i == 5 && j == 2
            temp = limit(J(i,j),xFCV,D0);
        else
            A(i,j) = subs(J(i,j),Y,[0 D0 0 0 0 0]);
        end
    
    end
end

eigA = eig(A)


