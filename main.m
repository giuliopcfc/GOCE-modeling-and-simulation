% Modeling and Simulation of Aerospace Systems (2020/2021)
% Project
% Authors: Giulio Pacifici, Lorenzo Porcelli, Giacomo Velo

clear;clc;close all;set(0,'defaultTextInterpreter','latex');

%% LOAD DATA:

config;

%% INTEGRATION:

%%% Initial conditions:
% Orbital Mechanics:
a0 = data.orbit.altitude + data.const.R_MEAN; % Initial Semi Major Axis [km] %%%% R_MEAN o R_EQUATORIAL?? %%%%

% Initial State for GPEs:
Y0GPE = [a0; data.orbit.eccentricity; data.orbit.inclination;...
        data.orbit.argPerigee; data.orbit.RAAN; data.orbit.theta0]; 
    
% Assembly initial state array:
Y0 = Y0GPE;

%%% Integration:
options = odeset('AbsTol',1e-14,'RelTol',1e-13);
[T,Y] = ode113(@odeFun,[0 1e5],Y0,options,data);

%% PLOT RESULTS:

plots;