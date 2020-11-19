function aJ2 = pertJ2(rr,k_E)
% pertJ2 Perturbing acceleration due to secular J2 effects
% 
% Function to compute the perturbing acceleration aJ2 = [aJ2x;aJ2y;aJ2z] 
% due to Earth's oblateness. This ccauses the gravitational field to depend
% not only distance, but also on latitude. These  zonal variations of the
% gravitational fieldca be expressed as a series.
%
% PROTOTYPE:
%  aJ2 = pertJ2(rr,k_E)
%  
% INPUT:
%  rr   [3,1]    Position vector in inertial frame [km]
%  k_E  [1]      Gravity constant of Earth [km^3/s^2]
% 
% OUTPUT:
%  aJ2 [3,1]     Perturbing acceleration vector due to secular J2 effects [km/s^2]
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
J2 = 0.00108263;      % J2 factor
Rpl_E = 6378.137;     % Equatorial radius of Earth

X = rr(1);
Y = rr(2);
Z = rr(3);
r = norm(rr);

aJ2 = 3/2*J2*k_E*Rpl_E^2/r^4*[X/r*(5*Z^2/r^2-1);... 
    Y/r*(5*Z^2/r^2-1); Z/r*(5*Z^2/r^2-3)];

end