%% Linearization:
% The linearization of the system is performed here. The equilibrium
% condition of the system is selected as the initial one.

% Definition of the syms variables:
syms IVOut xFCV vFCV xAcc vAcc VOut a e i OM om theta

symY = [IVOut xFCV vFCV xAcc vAcc VOut a e i OM om theta]; % State vector

symDY = symOdeFun(0,symY,data);

% NOTE: at least for our computers, the mere display of the symbolic 
% Jacobian took a very long time. We suggest not displaying it in its 
% integrity. If interested in seeing it, we rather suggest to do it 
% by diplaying single rows/colums one at the time.
symJ = jacobian(symDY,symY);                               % Jacobian matrix

A = double(subs(symJ,symY,data.ode.Y0'));                  % Jacobian matrix with numeriacal data

eigA = eig(A);

%% Symbolic Functions:

function  dYFCV = symFlowControlValve(YFCV,VOut,data)

% Load data:
kI = data.FCV.kI;
kProp = data.FCV.kProp; kInt = data.FCV.kInt;
m = data.FCV.massSpool; kSpring = data.FCV.kSpring; c = data.FCV.c;

% Load state variables:
IVOut = YFCV(1); xV = YFCV(2); vV = YFCV(3);

% PI controller:
I = kProp*VOut + kInt*IVOut;

x0 = data.FCV.x0; % Rest position of the spring [m]

% EOM of the spool:
dxV = vV;

dvV = 1/m*( kSpring*(x0 - xV) - kI*I - c*vV );

% Derivatives of the state variables:
dYFCV = [VOut; dxV; dvV];

end

function [aThrust] = symIonThruster(xV,data)

D0 = data.FCV.D0;

% Assigning the value to z
z = 1-xV/D0;

% Computation of the orifice area
alpha = 2*acos(1-2*z);
areaOrifice = D0^2/8*(alpha - sin(alpha));

% Since the pressure after the valve is much lower than the critical
% pressure, the flow is chocked. Hypothesis of homoentropic flow.
T2 = data.thruster.T2;
p2 = data.thruster.p2;
k =  data.thruster.kXe;
R = data.const.Ru/data.thruster.mmXe;

mDot = areaOrifice*p2*sqrt(k)/sqrt(T2*R)*(1+0.5*(k-1))^((-k-1)/2/(k-1));

vAcc = sqrt(2*data.thruster.e*data.thruster.deltaV/data.thruster.massIon);

aThrust = mDot*vAcc./data.goce.mass;

end

function [dYA] = symAccelerometer(YA, aThrust, aDragV, data)

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

% Derivatives of the state variables:
dYA = [dxA; dvA; dVOut];

end

function dYGPE = symGPE(YGPE,rr,vv,aDrag,aJ2,aThrust,data)

MU = data.const.MU_EARTH;

% Initialize keplerian elements:
a = YGPE(1); e = YGPE(2); i = YGPE(3);
om = YGPE(5); f = YGPE(6);

% Perturbing acceleration:
% Thrust:
aThrust = aThrust/1000; % [km/s^2]
% Drag:
aDrag = aDrag/1000; % [km/s^2]

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

end

function dY = symOdeFun(~,Y,data)

% Load data:
MU = data.const.MU_EARTH;

% Load state arrays:
YFCV = Y(1:3); YA = Y(4:6); YGPE = Y(7:12);

% Load state variables:
xV = YFCV(2);
VOut = YA(3);
a = YGPE(1);        e = YGPE(2);    i = YGPE(3);
OM = YGPE(4);       om = YGPE(5);   f = YGPE(6);

% Flow Control Valve:
dYFCV = symFlowControlValve(YFCV,VOut,data);

% Ion Thruster:
[aThrust] = symIonThruster(xV,data);

% From orbital parameters to cartesian coordinates:
[rr,vv] = kep2car(a,e,i,OM,om,f,MU);

% Orbital Perturbations:
% Drag perturbing acceleration:
rho = 6.6386e-11;  % Reference Density

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

aDragV = dot(aDrag,vv/norm(vv)); % Drag acceleration in velocity direction

% Accelerometer:
[dYA] = symAccelerometer(YA, aThrust, aDragV, data);

% Orbital Mechanics:
dYGPE = symGPE(YGPE,rr,vv,aDrag,aJ2,aThrust,data);

dY = [dYFCV; dYA; dYGPE];
end

