function input = inputLoad
% inputLoad
% 
% Function to load the input structure for assignment 2 into workspace.
% 
% PROTOTYPE:
%  [input] = inputLoad
% 
% INPUT:
% 
% OUTPUT:
%  input    Input structure
% 
% CONTRIBUTORS:
%  Lyle Campbell
%  Giulio Pacifici 
%  Luca Rizzieri 
%  Davide Sisana
% 
% VERSIONS:
%  02-11-2019: First version



%% Orbit Data:

% Orbit Parameters:

input.a0 = 1.8302*1e4; % Semi-major axis [km]
input.e0 = 0.6158; % Eccentricity 
input.i0 = deg2rad(71.5679); % Inclination [rad]
input.hp = 660.618; % Heigth of pericenter [km]

% SC Parameters:

input.cd = 2.2; % Drag Coefficient
input.Am = 0.06; % Area-Mass Ratio [m^2/kg]

% Repeating ground tracks:

input.m = 2; % Satellite revolutions
input.k = 7; % Earth rotations

%% Set initial parameters:

input.OM0 = deg2rad(20); % RAAN [rad]
input.om0 = deg2rad(70); % Anomaly of pericenter [rad]
input.f0 = 0; % True Anomaly [rad]

%% Other inputs:
input.k_E = astroConstants(13);
input.w_E = (2*pi + 2*pi/365.26)/24/3600*[0;0;1]; % Earth's angular velocity vector
input.T = 2*pi*sqrt(input.a0^3/input.k_E); % Orbital period 

input.Y0GPE = [input.a0;input.e0;input.i0;input.OM0;...
    input.om0;input.f0]; % Initial state array for GPE

input.Y0Cart = input.Y0GPE;
[input.Y0Cart(1:3),input.Y0Cart(4:6)] = par2car(input.a0,input.e0,input.i0,...
    input.OM0,input.om0,input.f0,input.k_E); % Initial state array for Cartesian
