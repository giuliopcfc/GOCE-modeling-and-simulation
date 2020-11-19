function [lonNaN,latNaN,alpha,delta,lon,lat] = GT(input)
% GT Ground track of a satellite 
% 
% Function to compute the satellite's ground track.
% 
% PROTOTYPE:
%  [lonNaN,latNaN,alpha,delta,lon,lat] = GT(tspan,y0,thetag0,mu)
%  
% INPUT:
%  input        Input structure
% 
% OUTPUT:
%  lonNaN    Vector of longitudes suitable for plots
%  latNaN    Vector of latitudes suitable for plots
%  alpha     Vector of right ascentions
%  delta     Vector of declinations
%  lon       Vector of longitudes 
%  lat       Vector of latitudes
% 
% CONTRIBUTORS:
%  Lyle Campbell
%  Giulio Pacifici 
%  Luca Rizzieri 
%  Davide Sisana
% 
% VERSIONS:
%  02-11-2019: First version
%  15-12-2019: ( Giulio Pacifici ) Changed integration function. Changed
%  name of the function.
%

% Orbit propagation:

input.method = 'GPE';
[t,~,cart] = orbitIntegration(input);

% Get cartesian coordinates:
x = cart(:,1);
y = cart(:,2);
z = cart(:,3);

r = sqrt(x.^2+y.^2+z.^2); % Norm of position vector at each time step [km]

delta = asin(z./r); % Declination [rad]

alpha = (y>0).*acos(x./r./cos(delta))+(y<=0).*...
    (2*pi - acos(x./r./cos(delta))); % Right ascention [rad]

lat = delta; % Latitude [rad]

we = norm(input.w_E); % Earth's rotation rate [rad/s]
lon = wrapToPi(alpha - we*t); % Longitude [rad] 

% Insert NaN every time the longitude vector goes from pi to -pi or from
% -pi to 180 pi:
latNaN = [lat(1)];
lonNaN = [lon(1)];
index = 1;
for i = 2:(length(lon)-1)
    if lon(i)>lon(i+1)&&lon(i)>lon(i-1)&&lon(i+1)<lon(i+2)
        value = interp1([lon(i);lon(i+1)+2*pi],[lat(i);lat(i+1)],pi);
        latNaN = [latNaN; lat(index+1:i);value;NaN;value];
        lonNaN = [lonNaN; lon(index+1:i);pi;NaN;-pi];
        index = i;
    elseif lon(i)<lon(i+1) && lon(i)<lon(i-1)&& lon(i+1)>lon(i+2)
        value = interp1([lon(i) lon(i+1)-2*pi],[lat(i) lat(i+1)],-pi);
        latNaN = [latNaN; lat(index+1:i);value;NaN;value];
        lonNaN = [lonNaN; lon(index+1:i);-pi;NaN;pi];
        index = i;
    end
end

% Convert radians to degrees:
latNaN = [latNaN;lat(index+1:end)]*180/pi;
lonNaN = [lonNaN;lon(index+1:end)]*180/pi;

end