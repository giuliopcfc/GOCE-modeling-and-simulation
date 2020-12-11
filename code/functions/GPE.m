function dYGPE = GPE(YGPE,rr,vv,aDrag,aJ2,aThrust,data)
% 
% GPE Gauss Planetary Equations. Function to compute the derivative of the 
% keplerian elements' state array.
%  
% INPUT:
%  YGPE  [6,1]      Keplerian elements array
%  rr [3,1]         Position vector in inertial frame [km]
%  vv [3,1]         Velocity vector in inertial frame [km/s]
%  aDrag [3,1]      Drag acceleration [m/s^2]
%  aJ2 [3,1]        J2 perturbing acceleration [km/s^2]
%  aThrust [3,1]    Thrust acceleration [m/s^2]
%  data             data struct
% 
% OUTPUT:
%  dYGPE [6,1]      Time derivative of the state vector
% 

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
