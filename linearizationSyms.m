% LINEARIZATION OF THE MATHEMATICAL MODEL

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %AGGIORNARE LE EQUATIONI SE CAMBIANO!!!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
syms intVOut xFCV vFCV xA vA VOut
Y = [intVOut xFCV vFCV xA vA VOut];

%% FCV:
% Declaration of symbolic variables:
syms kI kPropFCV kIntFCV mFCV kSpring c A0

D0 = sqrt(4*A0/pi); % Diameter of the orifice

% PI controller:
iFCV = kPropFCV*VOut + kIntFCV*intVOut;

x0 = D0; % Rest position of the spring [m]

% EOM of the spool:
dxFCV = vFCV;

dvFCV = 1/mFCV*( kSpring*(x0 - xFCV) - kI*iFCV - c*vFCV );

% Derivatives of the state variables:
dYFCV = [VOut, dxFCV, dvFCV];

%% Ion Thruster:
% Declaration of symbolic variables:
syms TX pX k R e DVT massIonX

% assigning the value to z
z = 1-xFCV/D0;

% computation of the orifice area
alpha = 2*acos(1-2*z);
areaOrifice = D0^2/8*(alpha - sin(alpha));

mDot = areaOrifice*pX*sqrt(k)/sqrt(TX*R)*(1+0.5*(k-1))^((-k-1)/2/(k-1));

vAcc = sqrt(2*e*DVT/massIonX);

thrust = mDot*vAcc;

%% Accelerometer:
% Declaration of symbolic variables:
syms mGOCE areaA massA perm g VBias CF kPropA kDerA
syms dragV

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
VC = kPropA*VOut + kDerA*dVOut;

% Electrostatic forces acting on the mass:
DV1 = VBias - VC - 1/2*Vx;
DV2 = VBias + VC + 1/2*Vx;
F1 = 0.5*C1*DV1^2/(g - xA);
F2 = 0.5*C2*DV2^2/(g + xA);

% EOM of the seismic mass:
dxA = vA;
dvA = a_ext + (-F1 + F2)/massA;

%% Full system:

% Load data:
config;
goce = data.goce;
accelerometer = data.accelerometer;
FCV = data.FCV;
thruster = data.thruster;
const = data.const;

% Initial state:
Y0 = [0 sqrt(4*data.FCV.A0/pi) 0 0 0 0];

% Derivatives of the state variables:
dYA = [dxA, dvA, dVOut];
dY = [dYFCV, dYA];

% Matrix A:
JA = jacobian(dY,Y);

data_vecA = [FCV.kI, FCV.kProp, FCV.kInt, FCV.massSpool, FCV.kSpring, FCV.c,...
    FCV.A0, thruster.T2, thruster.p2, thruster.kXe, const.Ru, thruster.e,...
    thruster.deltaV, thruster.massIon, goce.mass, accelerometer.areaMass,...
    accelerometer.mass, accelerometer.eps, accelerometer.gap, accelerometer.VBias,...
    accelerometer.C, accelerometer.kProp, accelerometer.kDer];
Atemp = subs(JA, [kI, kPropFCV, kIntFCV, mFCV, kSpring, c, A0, TX, pX, k, R,...
    e, DVT, massIonX, mGOCE, areaA, massA, perm, g, VBias, CF, kPropA,...
    kDerA], data_vecA);

A = simplify(subs(Atemp, Y, Y0));

% Matrix B:
JB = jacobian(dY, dragV);
B = subs(JB, goce.mass);

% Matrix C:
JC = jacobian(thrust, Y);

data_vecC = [FCV.A0, thruster.deltaV, const.Ru, thruster.T2, thruster.e,...
    thruster.kXe, thruster.massIon, thruster.p2];
Ctemp = subs(JC, [A0, DVT, R, TX, e, k, massIonX, pX], data_vecC);

C = simplify(subs(Ctemp, xFCV, Y0(2)));

% Eigenvalues of the linear system:
eigA = eig(A)

% figure;
% scatter(real(eigA), imag(eigA), 25, 'filled')
% xlabel('Re{\lambda}')
% ylabel('Im{\lambda}')
% grid on
