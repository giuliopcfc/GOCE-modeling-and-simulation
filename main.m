% Modeling and Simulation of Aerospace Systems (2020/2021)
% Project
% Authors: Giulio Pacifici, Lorenzo Porcelli, Giacomo Velo

clear;clc;close all;

%% LOAD DATA:

config;

%% INTEGRATION:

%%% Initial conditions:
% Initial State for flow control valve:
Y0FCV = [0; 0; 0];

% Initial State for accelerometer:
Y0A = [0; 0; 0];

% Orbital Mechanics:
a0 = data.orbit.altitude + data.const.R_MEAN; % Initial Semi Major Axis [km] %%%% R_MEAN o R_EQUATORIAL?? %%%%

% Initial State for GPEs:
Y0GPE = [a0; data.orbit.eccentricity; data.orbit.inclination*pi/180;...
        data.orbit.argPerigee*pi/180; data.orbit.RAAN*pi/180; data.orbit.theta0*pi/180]; 
    
% Assembly initial state array:
Y0 = [Y0A; Y0FCV; Y0GPE];

%%% Integration:
options = odeset('AbsTol',1e-10,'RelTol',1e-8);
[T,Y] = ode15s(@odeFun,[0 1e5],Y0,options,data);

%% PLOT RESULTS:

plots;