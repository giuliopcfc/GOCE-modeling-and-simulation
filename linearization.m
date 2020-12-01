syms intVOut xFCV vFCV xAcc vAcc VOut a e i OM om theta 

Y = [intVOut xFCV vFCV xAcc vAcc VOut a e i OM om theta];

dY = symOdeFun(0,Y,data);

J = jacobian(dY,Y);

A = double(subs(J,Y,data.ode.Y0(1:12)'));

eigA = eig(A); 

%% Symbolic Functions:
function  dYFCV = symFlowControlValve(YFCV,VOut,data)

% Load data:
kI = data.FCV.kI;
kProp = data.FCV.kProp; kInt = data.FCV.kInt;
m = data.FCV.massSpool; kSpring = data.FCV.kSpring; c = data.FCV.c;

% Load state variables:
intVOut = YFCV(1); x = YFCV(2); v = YFCV(3);

% PI controller:
i = kProp*VOut + kInt*intVOut;

x0 = data.FCV.x0; % Rest position of the spring [m]

% EOM of the spool:
dx = v;

dv = 1/m*( kSpring*(x0 - x) - kI*i - c*v );

% Derivatives of the state variables:
dYFCV = [VOut; dx; dv];

end

function [thrust] = symIonThruster(xFCV,data) 
D0 = data.FCV.D0;

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

end

function [dYA] = symAccelerometer(YA, thrust, dragV, data)

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

% Derivatives of the state variables:
dYA = [dx; dv; dVOut];

end

function dYGPE = symGPE(YGPE,rr,vv,aDrag,thrust,data)

MU = data.const.MU_EARTH;

% Initialize keplerian elements:
a = YGPE(1); e = YGPE(2); i = YGPE(3);
OM = YGPE(4); om = YGPE(5); f = YGPE(6);

% Perturbing acceleration:
% Thrust:
aThrust = thrust/data.goce.mass/1000; % [km/s^2]

% J2 Effect:
aJ2 = pertJ2(rr,data);

% Rotation to TNH frame:
tVers = vv/norm(vv);                       % Tangential versor
hVers = cross(rr,vv)/norm(cross(rr,vv));   % Out of plane versor
nVers = cross(hVers,tVers);                % Normal versor
A = [tVers,nVers,hVers];                   % Rotation Matrix

% Convert aDrag to [km/s^2]:
aDrag = aDrag/1000;

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

function dY = symOdeFun(t,Y,data)
% Load data:
MU = data.const.MU_EARTH;

% Load state arrays:
YFCV = Y(1:3); YA = Y(4:6); YGPE = Y(7:12);

% Flow Control Valve:
dYFCV = symFlowControlValve(YFCV,YA(3),data);

% Ion Thruster:
[thrust] = symIonThruster(YFCV(2),data);

% Initialize keplerian elements:
a = YGPE(1); e = YGPE(2); i = YGPE(3);
OM = YGPE(4); om = YGPE(5); f = YGPE(6);

% From orbital parameters to cartesian coordinates:
[rr,vv] = kep2car(a,e,i,OM,om,f,MU);  

% Aerodynamic Drag:
vRel = (vv - cross(data.const.W_EARTH*[0;0;1],rr))*1000; % Air Relative Velocity [m/s]

rho = 6.6386e-11;  % Reference Density

aDrag = -0.5/data.goce.balCoeff*rho*norm(vRel)*vRel; % Perturbing acceleration [m/s^2]
             
% Drag component in the direction of velocity:
dragV = data.goce.mass*dot(aDrag,vv/norm(vv));

% Accelerometer:
[dYA] = symAccelerometer(YA, thrust, dragV, data);

% Orbital Mechanics:
dYGPE = symGPE(YGPE,rr,vv,aDrag,thrust,data);

dY = [dYFCV; dYA; dYGPE];
end