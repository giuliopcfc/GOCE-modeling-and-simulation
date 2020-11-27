% Modeling and Simulation of Aerospace Systems (2020/2021)
% Project
% Authors: Giulio Pacifici, Lorenzo Porcelli, Giacomo Velo

clear;clc;close all;

%% Load Data:

config;

%% Integration:

% Set the initial position of the flow control valve and the equilibrium
% position of the spring such that the thrust equals the drag at t = 0:
[~,out] = odeFun(0,data.ode.Y0,data);
%%%%%%%%%JACK%%%%%%%%%%%%%

% Time span array:
tspan = [0 2*data.orbit.period];

% Integration:
options = odeset('AbsTol',1e-14,'RelTol',1e-13);

tic
[T,Y,out] = integrateOdeFun(@odeFun, tspan, data.ode.Y0, options, data);
toc

%% Plots:

plots;