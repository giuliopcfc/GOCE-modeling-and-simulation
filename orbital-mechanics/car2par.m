function [a,e,i,OM,om,th,kep] = car2par(rr,vv,mu)
% car2par Cartensian coordinates to orbital parameters
% 
% Function to compute the orbital parameters given position and velocity
% vectors of the satellite.
% 
% PROTOTYPE:
%  [a,e,i,OM,om,th] = car2par(rr,vv,mu)
%  
% INPUT:
%  rr [3,1]    Position vector in inertial frame [km]
%  vv [3,1]    Velocity vector in inertial frame [km/s]
%  mu [1]      Gravitational parameter of the primary [km^3/s^2]
% 
% OUTPUT:
%  a [1]       Semimajor axis [km]
%  e [1]       Eccentricity [-]
%  i [1]       Inclination angle [rad]
%  OM [1]      Right ascention of the ascending node [rad]
%  om [1]      Anomaly of the pericenter [rad]
%  th [1]      True anomaly [rad]
%  kep [6,1]   Keplerian elements array 
% 
% CONTRIBUTORS:
%  Lyle Campbell
%  Giulio Pacifici 
%  Luca Rizzieri 
%  Davide Sisana
% 
% VERSIONS:
%  01-11-2019: First version
% 
k = [ 0 0 1]';

r = norm(rr);
v = norm(vv);

a = 1/(2/r - (v^2)/mu);

hh = cross(rr,vv);
h = norm(hh);

ee = cross(vv,hh)/mu - rr/r;
e = norm(ee);

i = acos(hh(3)/h);

% If inclination is 0 rad, ascending node N is not defined. We assume it to
% be the versor in x direction.
if i == 0
    N = [1;0;0]; 
    if ee(2) >= 0
        om = acos(dot(N,ee)/e);
    else
        om = 2*pi - acos(dot(N,ee)/e);
    end
else
    N = cross(k,hh);
    N = N/norm(N);   
    if ee(3) >= 0
        om = acos(dot(N,ee)/e);
    else
        om = 2*pi - acos(dot(N,ee)/e);
    end
end

if N(2) >= 0 
    OM = acos(N(1));
else
    OM = 2*pi - acos(N(1));
end

vr = dot(vv,rr)/r;

if vr >= 0 
    th = real(acos(dot(rr,ee)/r/e));
elseif vr < 0
    th = 2*pi - real(acos(dot(rr,ee)/r/e));
end

kep = [a;e;i;OM;om;th]; 


