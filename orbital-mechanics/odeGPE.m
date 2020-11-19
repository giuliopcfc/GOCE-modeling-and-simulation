function dY = odeGPE(t,Y,input)
% odeGPE Gauss Planetary Equations ode function
% 
% Input function for the ode integration of the two-body problem made by
% Keplerian elements array Y = [a;e;i;OM;om;f].
% 
% PROTOTYPE:
%  dY = odeGPE(~,Y,input)
%  
% INPUT:
%  Y     [6,1]      Keplerian elements array
%  input [1]        Input parameters:
%                   - k_E  Gravity constant of Earth [km^3/s^2]
%                   - w_e  Earth's rotation velocity (eastwards) [deg/s]
%                   - Am   Ratio between Reference area and Spacecraft mass [m^2/kg]
%                   - cd   Drag coefficient [-]
% 
% OUTPUT:
%  dY    [6,1]      Time derivative of the state vector [km/s] and [km/s^2]
% 
% CONTRIBUTORS:
%  Lyle Campbell
%  Giulio Pacifici 
%  Luca Rizzieri 
%  Davide Sisana
% 
% VERSIONS:
%  02-11-2019: First version
%

k_E = input.k_E;
w_E = input.w_E;
Am = input.Am;
cd = input.cd;

% Initialize keplerian elements:
a = Y(1); e = Y(2); i = Y(3); OM = Y(4); om = Y(5); f = Y(6);

% Orbit parameters:
b = a*sqrt(1-e^2); 
p = b^2/a; 
r = p/(1+e*cos(f));
v = sqrt(2*k_E/r - k_E/a);
n = sqrt(k_E/a^3);
h = n*a*b;
fs = f + om;

% Passage from orbital parameters to cartesian coordinates:
[rr,vv] = par2car(a,e,i,OM,om,f,k_E);  

% Get the perturbing acceleration:
ap = [0;0;0];

switch input.type
    case 0                           % unperturbed motion
        ap = [0;0;0];
    case 1                           % only secular J2 effects perturbation
        aJ2 = pertJ2(rr,k_E);
        ap = aJ2;
    case 2                           % only aerodynamic drag perturbation
        vrel = vv - cross(w_E,rr);
        aD = pertD(Am,cd,vrel,r);
        ap = aD;
    case 3                           % J2+Drag perturbation
        aJ2 = pertJ2(rr,k_E);
        vrel = vv - cross(w_E,rr);
        aD = pertD(Am,cd,vrel,r);
        ap = aJ2 + aD;     
end


A = car2tnh(rr,vv);     % Rotation matrix
ap = A*ap;
at = ap(1);             % Tangential component of perturbing acceleration
an = ap(2);             % Normal component of perturbing acceleration
ah = ap(3);             % Out of plane component of perturbing acceleration

% Gauss planetary equations:

da = 2*a^2*v/k_E*at;

de = 1/v*(2*(e+cos(f))*at - r/a*sin(f)*an);

di = r*cos(fs)/h*ah;

dOM = r*sin(fs)/h/sin(i)*ah;

dom = 1/e/v*(2*sin(f)*at + (2*e + r/a*cos(f))*an) - ...
    r*sin(fs)*cos(i)/h/sin(i)*ah;

df = h/r^2 - 1/e/v*(2*sin(f)*at + (2*e + r/a*cos(f))*an);

% Assembling array of state derivatives:
dY = [da;de;di;dOM;dom;df];

