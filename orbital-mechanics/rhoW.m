function rho = rhoW(h)
% rhoW
% 
% Function to determine the atmosphere density at a given altitude with the
% exponential atmospheric model from Wertz.
% 
% PROTOTYPE:
%  rho = rhoW(h)
% 
% INPUT:
%  h         Altitude [km]
% 
% OUTPUT:
%  rho       Atmosphere density [kg/m^3]
% 
% CONTRIBUTORS:
%  Lyle Campbell
%  Giulio Pacifici 
%  Luca Rizzieri 
%  Davide Sisana
% 
% VERSIONS:
%  02-11-2019: First version

if h<=300
    h0 = 250;
    rho0 = 7.248*1e-11;
    H = 45.546;
elseif h<=350 && h>300
    h0 = 300;
    rho0 = 2.418*1e-11;
    H = 53.628;
elseif h<=400 && h>350
    h0 = 350;
    rho0 = 9.158*1e-12;
    H = 53.298;
elseif h<=450 && h>400
    h0 = 400;
    rho0 = 3.725*1e-12;
    H = 58.515;
elseif h<=500 && h>450
    h0 = 450;
    rho0 = 1.585*1e-12;
    H = 60.828;
elseif h<=600 && h>500
    h0 = 500;
    rho0 = 6.967*1e-13;
    H = 63.822;
elseif h<=700 && h>600
    h0 = 600;
    rho0 = 1.454*1e-13;
    H = 71.835;
elseif h<=800 && h>700
    h0 = 700;
    rho0 = 3.614*1e-14;
    H = 88.667;
elseif h<=900 && h>800
    h0 = 800;
    rho0 = 1.170*1e-14;
    H = 124.64;
elseif h<=1000 && h>900
    h0 = 900;
    rho0 = 5.245*1e-15;
    H = 181.05;
elseif h>1000
    h0 = 1000;
    rho0 = 3.019*1e-15;
    H = 268;
end

rho = rho0*exp(-(h-h0)/H);

if h>1700
    rho = 0;
end

end
