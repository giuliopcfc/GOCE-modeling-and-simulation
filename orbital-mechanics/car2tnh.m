function A = car2tnh(rr,vv)
% car2tnh Cartesian coordinates to (t,n,h) reference system
% 
% Function to compute the rotation matrix from Cartesian coordinates to
% directions of perturbing acceleration: a_tnh = A * a_car.
% 
% PROTOTYPE:
%  A = car2tnh(rr,vv)
%  
% INPUT:
%  rr [3,1]    Position vector in inertial frame [km]
%  vv [3,1]    Velocity vector in inertial frame [km/s]
% 
% OUTPUT:
%  A [3,3]     Rotation matrix [-]
% 
% CONTRIBUTORS:
%  Lyle Campbell
%  Giulio Pacifici 
%  Luca Rizzieri 
%  Davide Sisana
% 
% VERSIONS:
%  04-12-2019: First version
% 
t = vv/norm(vv);                       % Tangential versor

h = cross(rr,vv)/norm(cross(rr,vv));   % Out of plane versor

n = cross(h,t);                        % Normal versor

A = [t,n,h]';