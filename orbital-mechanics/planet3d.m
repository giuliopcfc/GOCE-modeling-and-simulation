function planet3d(id, position, fig_handle, sf)
%
% planet3d.m
% 
% Function to plot the 3d planet into a figure.
% 
% PROTOTYPE:
%  planet3d(id, position, fig_handle, sf)
%  
% INPUT:
%  id           Planet id number ( 3 for Earth, 4 for Mars, 8 for Neptune)
%  position     Planet position in space
%  fig_handle   Figure handle
%  sf           Scale factor
% 
% OUTPUT:
% 
% CONTRIBUTORS:
%  Lyle Campbell
%  Giulio Pacifici 
%  Luca Rizzieri 
%  Davide Sisana
% 
% VERSIONS:
%  01-02-2019: First version
% 

% Select planet:
switch id
    case 3
        name = 'EarthTexture.jpg';        % Load planet texture
        R_pl = astroConstants(23);         % Planet radius
    case 4
        name = 'MarsTexture.jpg';
        R_pl = astroConstants(24);
    case 8
        name = 'NeptuneTexture.jpg';
        R_pl = astroConstants(28);
end

% Build a sphere:
axis_handle = gca(fig_handle);             % Get axis handle

[x,y,z] = sphere(100);                     % Get sphere surface coordinates

% Set planet position, introduce scale factor and planet radius:
x = sf*R_pl*x + position(1);
y = sf*R_pl*y + position(2);
z = sf*R_pl*z + position(3);

% Plot:
planet = surf(axis_handle,x,y,z);          % Plot sphere

cdata = imread(name);                      % Get texture image data

% Put texture over the sphere:
planet.FaceColor = 'texturemap';
planet.EdgeColor = 'none';
planet.CData = flip(cdata);
