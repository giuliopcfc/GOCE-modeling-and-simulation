function [rr,vv] = par2car(a,e,i,OM,om,th,mu)
% par2car Orbital parameters to cartesian coordinates
% 
% Function to compute position and velocity vectors of the satellite the 
% orbital parameters given the orbital parameters.
% 
% PROTOTYPE:
%  [rr,vv] = par2car(a,e,i,OM,om,th,mu)
%  
% INPUT:
%  a [1]       Semimajor axis [km]
%  e [1]       Eccentricity [-]
%  i [1]       Inclination angle [rad]
%  OM [1]      Right ascention of the ascending node [rad]
%  om [1]      Anomaly of the pericenter [rad]
%  th [1]      True anomaly [rad]
%  mu [1]      Gravitational parameter of the primary [km^3/s^2]
% 
% OUTPUT:
%  rr [3,1]    Position vector in inertial frame [km]
%  vv [3,1]    Velocity vector in inertial frame [km/s]
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
p = a*(1-e^2);
r = p/(1+e*cos(th));

rr = r*[cos(th);sin(th);0];
vv = sqrt(mu/p)*[-sin(th);e+cos(th);0];

ROM = [ cos(OM) sin(OM) 0; -sin(OM) cos(OM) 0; 0 0 1];
Ri = [ 1 0 0; 0 cos(i) sin(i); 0 -sin(i) cos(i)];
Rom = [ cos(om) sin(om) 0; -sin(om) cos(om) 0; 0 0 1];

T = Rom*Ri*ROM;

rr = T'*rr;
vv = T'*vv;


