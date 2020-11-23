% Modeling and Simulation of Aerospace Systems (2020/2021)
% Project
% Authors: Giulio Pacifici, Lorenzo Porcelli, Giacomo Velo

clear;clc;close all;

%% LOAD DATA:

config;

%% INTEGRATION:

%%% Initial conditions:
% Orbital Mechanics:
a0 = data.orbit.altitude + data.const.R_MEAN; % Initial Semi Major Axis [km] %%%% R_MEAN o R_EQUATORIAL?? %%%%

% Initial State for flow control valve:
Y0FCV = [0; 0; 0];

% Initial State for GPEs:
Y0GPE = [a0; data.orbit.eccentricity; data.orbit.inclination;...
        data.orbit.argPerigee; data.orbit.RAAN; data.orbit.theta0]; 
    
% Assembly initial state array:
Y0 = [Y0FCV;Y0GPE];

%%% Integration:
options = odeset('AbsTol',1e-10,'RelTol',1e-8);
[T,Y] = ode15s(@odeFun,[0 1e5],Y0,options,data);

%% PLOT RESULTS:

plots;