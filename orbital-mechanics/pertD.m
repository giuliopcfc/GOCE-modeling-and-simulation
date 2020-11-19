function aD = pertD(Am,cd,vrel,r)
% pertD Perturbing acceleration due to aerodynamic drag
% 
% Function to compute the perturbing acceleration aD = [aDx;aDy;aDz]
% due to aerodynamic drag force on an orbit spacecraft through some 
% parameters that come from spacecraft configuration (A_cross, m, cd) 
% and atmospheric characteristics (vrel, rho).  
% 
% PROTOTYPE:
%  aD = pertD(Am,cd,vrel,r)
%  
% INPUT:
%  Am   [1]    Ratio between Reference area (A_cross) and Spacecraft mass (m) [m^2/kg]
%  cd   [1]    Drag coefficient [-]
%  vrel [3,1]  Air-relative speed vector [km/s]
%  r    [1]    Geocentric distance [km] 
% 
% OUTPUT:
%  aD [3,1]    Perturbing acceleration vector due to aerodynamic drag [km/s^2]
% 
% CONTRIBUTORS:
%  Lyle Campbell
%  Giulio Pacifici 
%  Luca Rizzieri 
%  Davide Sisana
% 
% VERSIONS:
%  04-12-2019: First version
%  15-12-2019: ( Giulio Pacifici ) Added rhoWUS function to determine
%  density
% 
R_E = astroConstants(23); % Earth radius [km]
H = r - R_E; % Heigth of satellite [km]
rho =  rhoW(H); % Density [kg/m^3]

aD = -0.5*Am*cd*rho*norm(vrel)*vrel;
aD = aD*1000;                    

end
