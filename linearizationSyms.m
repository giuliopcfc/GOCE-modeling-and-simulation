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

% Derivatives of the state variables:
dYA = [dxA, dvA, dVOut];

dY = [dYFCV, dYA];

J = jacobian(dY,Y);

% Load data:
config;

% Substitution of numerical values:
vec = [data.FCV.kI, data.FCV.kProp, data.FCV.kInt, data.FCV.massSpool,...
    data.FCV.kSpring, data.FCV.c, data.FCV.A0, data.thruster.T2,...
    data.thruster.p2, data.thruster.kXe, data.const.Ru, data.thruster.e,...
    data.thruster.deltaV, data.thruster.massIon, data.goce.mass,...
    data.accelerometer.areaMass, data.accelerometer.mass,...
    data.accelerometer.eps, data.accelerometer.gap, data.accelerometer.VBias,...
    data.accelerometer.C, data.accelerometer.kProp, data.accelerometer.kDer];

Atemp = subs(J, [kI, kPropFCV, kIntFCV, mFCV, kSpring, c, A0, TX, pX, k, R,...
    e, DVT, massIonX, mGOCE, areaA, massA, perm, g, VBias, CF, kPropA,...
    kDerA], vec);
A = simplify(subs(Atemp, Y, [0 sqrt(4*data.FCV.A0/pi) 0 0 0 0]));

% Eigenvalues of the linear system:
eigA = eig(A)

figure;
scatter(real(eigA), imag(eigA), 25, 'filled')
xlabel('Re{\lambda}')
ylabel('Im{\lambda}')
grid on
