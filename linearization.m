% LINEARIZATION OF THE MATHEMATICAL MODEL

%{
dY1 = A1*Y1 + B1*dragV
thrust = C1*Y1

dY2 = A2*Y2 + B2*thrust
dragV = C2*Y2

dY1 = A1*Y1 + B1*C2*Y2
dY2 = A2*Y2 + B2*C1*Y1

Y = [Y1;Y2]
A = [  A1      B1*C2
       B2*C1      A2  ];
%}

config;

%% Flow Control Valve + Accelerometer:

syms intVOut xFCV vFCV xA vA VOut
Y1 = [intVOut xFCV vFCV xA vA VOut];

%%%%%%%%%%%%%%%%%%%%%%%%%%%% FCV %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Declaration of symbolic variables:
syms kI kPropFCV kIntFCV mFCV kSpring c

% PI controller:
iFCV = kPropFCV*VOut + kIntFCV*intVOut;

% EOM of the spool:
dxFCV = vFCV;

dvFCV = 1/mFCV*( kSpring*(data.FCV.x0 - xFCV) - kI*iFCV - c*vFCV );

% Derivatives of the state variables:
dYFCV = [VOut, dxFCV, dvFCV];

%%%%%%%%%%%%%%%%%%%%%%%% Ion Thruster %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Declaration of symbolic variables:
syms A0 TX pX k R e DVT massIonX

D0 = sqrt(4*A0/pi); % Diameter of the valve orifice

% assigning the value to z
z = 1-xFCV/D0;

% computation of the orifice area
alpha = 2*acos(1-2*z);
areaOrifice = D0^2/8*(alpha - sin(alpha));

mDot = areaOrifice*pX*sqrt(k)/sqrt(TX*R)*(1+0.5*(k-1))^((-k-1)/2/(k-1));

vAcc = sqrt(2*e*DVT/massIonX);

thrust = mDot*vAcc;

%%%%%%%%%%%%%%%%%%%%%%%% Accelerometer %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

%%%%%%%%%%%%%%%%%%%%%%%% FCV + Accelerometer %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Load data:
goce = data.goce;
accelerometer = data.accelerometer;
FCV = data.FCV;
thruster = data.thruster;
const = data.const;

% Initial state:
Y0 = data.ode.Y0(1:6)';

% Derivatives of the state variables:
dYA = [dxA, dvA, dVOut];
dY = [dYFCV, dYA];

% Matrix A:
JA = jacobian(dY,Y1);

data_vecA = [FCV.kI, FCV.kProp, FCV.kInt, FCV.massSpool, FCV.kSpring, FCV.c,...
    FCV.A0, thruster.T2, thruster.p2, thruster.kXe, const.Ru, thruster.e,...
    thruster.deltaV, thruster.massIon, goce.mass, accelerometer.areaMass,...
    accelerometer.mass, accelerometer.eps, accelerometer.gap, accelerometer.VBias,...
    accelerometer.C, accelerometer.kProp, accelerometer.kDer];
Atemp = subs(JA, [kI, kPropFCV, kIntFCV, mFCV, kSpring, c, A0, TX, pX, k, R,...
    e, DVT, massIonX, mGOCE, areaA, massA, perm, g, VBias, CF, kPropA,...
    kDerA], data_vecA);

A1 = subs(Atemp, Y1, Y0);

A1(5,2) = subs(limit(Atemp(5,2),Y1(2),Y0(2)),Y1,Y0);

% Matrix B:
JB = jacobian(dY, dragV);
B1 = subs(JB, goce.mass);

% Matrix C:
JC = jacobian(thrust, Y1);

data_vecC = [FCV.A0, thruster.deltaV, const.Ru, thruster.T2, thruster.e,...
    thruster.kXe, thruster.massIon, thruster.p2];
Ctemp = subs(JC, [A0, DVT, R, TX, e, k, massIonX, pX], data_vecC);

C1 = simplify(subs(Ctemp, xFCV, Y0(2)));

%% Gauss Planetary equations:

YGPE0 = data.ode.Y0GPE;

% Compute B2:

% Variations on thrust:
T0 = 0;
deltaT = 1e-3;
T_p = T0 + deltaT; T_m = T0 - deltaT;

% Variations of RHS of GPE:
[dYGPE_p] = funGPE(YGPE0,T_p,data);
[dYGPE_m] = funGPE(YGPE0,T_m,data);

% Central difference scheme
B2 = (dYGPE_p - dYGPE_m)/2/deltaT;

% Compute A and C with central difference scheme:
A2 = nan(6,6); C2 = nan(1,6);

for i = 1:6
    e = max(sqrt(eps),sqrt(eps)*abs(YGPE0(i))); % increment of x(i)
    x_p = YGPE0;    x_p(i) = YGPE0(i) + e;
    x_m = YGPE0;    x_m(i) = YGPE0(i) - e;
    
    
    [dYGPE_p, dragV_p] = funGPE(x_p,0,data);
    [dYGPE_m, dragV_m] = funGPE(x_m,0,data);
    
    A2(:,i) = (dYGPE_p - dYGPE_m)/2/e;
    
    C2(:,i) = (dragV_p - dragV_m)/2/e;
end

%% Augmented System:

A = [  A1         B1*C2
       B2*C1      A2    ];

A = double(A);

format shortE
eigA = eig(A)

clearvars -except A eigA

%% Functions:

function [dYGPE, dragV] = funGPE(YGPE,thrust,data)
%
% GPE Gauss Planetary Equations. Function to compute the derivative of the
% keplerian elements' state array and the drag force component along.
%
% INPUT:
%  YGPE  [6,1]      Keplerian elements array
%  rr [3,1]         Position vector in inertial frame [km]
%  vv [3,1]         Velocity vector in inertial frame [km/s]
%  thrust           Value of thrust [N]
%  data             data struct
%
% OUTPUT:
%  dYGPE [6,1]      Time derivative of the state vector
%

% Load data:
MU = data.const.MU_EARTH;

% Initialize keplerian elements:
a = YGPE(1); e = YGPE(2); i = YGPE(3);
OM = YGPE(4); om = YGPE(5); f = YGPE(6);

% Perturbing acceleration:
% Passage from orbital parameters to cartesian coordinates:
[rr,vv] = kep2car(a,e,i,OM,om,f,MU);

% Thrust:
aThrust = thrust/data.goce.mass/1000; % [km/s^2]

% Aerodynamic Drag:
aDrag = pertDrag(rr,vv,data);

% Convert aDrag to [km/s^2]:
aDrag = aDrag/1000;

% J2 Effect:
aJ2 = pertJ2(rr,data);

% Rotation to TNH frame:
tVers = vv/norm(vv);                       % Tangential versor
hVers = cross(rr,vv)/norm(cross(rr,vv));   % Out of plane versor
nVers = cross(hVers,tVers);                % Normal versor
A = [tVers,nVers,hVers];                   % Rotation Matrix

aPert = A'*(aDrag + aJ2) + [aThrust; 0; 0];

at = aPert(1);          % Tangential component of perturbing acceleration
an = aPert(2);          % Normal component of perturbing acceleration
ah = aPert(3);          % Out of plane component of perturbing acceleration

% Orbit parameters:
b = a*sqrt(1-e^2);
p = b^2/a;
r = p/(1+e*cos(f));
v = sqrt(2*MU/r - MU/a);
n = sqrt(MU/a^3);
h = n*a*b;
fs = f + om;

% Gauss Planetary Equations:
da = 2*a^2*v/MU*at;

de = 1/v*(2*(e+cos(f))*at - r/a*sin(f)*an);

di = r*cos(fs)/h*ah;

dOM = r*sin(fs)/h/sin(i)*ah;

dom = 1/e/v*(2*sin(f)*at + (2*e + r/a*cos(f))*an) - ...
    r*sin(fs)*cos(i)/h/sin(i)*ah;

df = h/r^2 - 1/e/v*(2*sin(f)*at + (2*e + r/a*cos(f))*an);

% Assembling array of state derivatives:
dYGPE = [da;de;di;dOM;dom;df];

% Drag component in the direction of velocity:
dragV = data.goce.mass*dot(aDrag,vv/norm(vv));
end



