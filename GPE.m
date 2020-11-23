function dY = GPE(Y,thrust,data)
% 
% GPE Gauss Planetary Equations. Function to compute the derivative of the 
% keplerian elements' state array.
%  
% INPUT:
%  Y     [6,1]      Keplerian elements array
%  thrust           Value of thrust [N]
%  data             data struct
% 
% OUTPUT:
%  dY    [6,1]      Time derivative of the state vector
% 

MU = data.const.MU_EARTH;

% Initialize keplerian elements:
a = Y(1); e = Y(2); i = Y(3); OM = Y(4); om = Y(5); f = Y(6);

% Orbit parameters:
b = a*sqrt(1-e^2); 
p = b^2/a; 
r = p/(1+e*cos(f));
v = sqrt(2*MU/r - MU/a);
n = sqrt(MU/a^3);
h = n*a*b;
fs = f + om;

% Passage from orbital parameters to cartesian coordinates:
[rr,vv] = kep2car(a,e,i,OM,om,f,MU);  

% Perturbing acceleration:

% Aerodynamic Drag:
aDrag = pertDrag(rr,vv,data);
% J2 Effect:
aJ2 = pertJ2(rr,data);

% Rotation to TNH frame:
tVers = vv/norm(vv);                       % Tangential versor
hVers = cross(rr,vv)/norm(cross(rr,vv));   % Out of plane versor
nVers = cross(hVers,tVers);                % Normal versor
A = [tVers,nVers,hVers];                   % Rotation Matrix

aPert = A'*(aDrag + aJ2) + [thrust; 0; 0];

at = aPert(1);          % Tangential component of perturbing acceleration
an = aPert(2);          % Normal component of perturbing acceleration
ah = aPert(3);          % Out of plane component of perturbing acceleration

% Gauss Planetary Equations:
da = 2*a^2*v/MU*at;

de = 1/v*(2*(e+cos(f))*at - r/a*sin(f)*an);

di = r*cos(fs)/h*ah;

dOM = r*sin(fs)/h/sin(i)*ah;

dom = 1/e/v*(2*sin(f)*at + (2*e + r/a*cos(f))*an) - ...
    r*sin(fs)*cos(i)/h/sin(i)*ah;

df = h/r^2 - 1/e/v*(2*sin(f)*at + (2*e + r/a*cos(f))*an);

% Assembling array of state derivatives:
dY = [da;de;di;dOM;dom;df];
