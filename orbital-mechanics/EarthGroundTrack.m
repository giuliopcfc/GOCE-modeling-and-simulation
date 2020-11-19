function EarthGroundTrack
% EarthGroundTrack
% 
% Function to set EarthTexture.jpg as background of the ground track plot.
% 
% PROTOTYPE:
%  EarthGroundTrack
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
I = imread('EarthTexture.jpg');
h = image([-180 180],[90 -90],I);
axis([-180 180 -90 90])
set(gca,'xtick',[-180 -90 0 90 180],'ytick',[-90 -45 0 45 90])
uistack(h,'bottom')
xlabel('Longitude [deg]'),ylabel('Latitude [deg]')
end
