% Modeling and Simulation of Aerospace Systems (2020/2021)
% Project
% Authors: Giulio Pacifici, Lorenzo Porcelli, Giacomo Velo
clear;clc;close all;
set(0,'defaultTextInterpreter','latex','defaultAxesFontSize',15);
set(0,'defaultAxesTickLabelInterpreter','latex');
set(0, 'defaultLegendInterpreter','latex');

%% Simulation type:

simType.main = 1; % Main simulation

simType.failures = 0; % Simulation with failures

simType.sensitivity = 0; % Sensitivity Analysis

simType.integrationAnalysis = 0; % Integration Analysis

simType.optimisation = 0; % Optimisation

simType.linearization = 0; % Linearization

%% Check on linked simulation types:

if simType.integrationAnalysis
    simType.linearization = 1;
end
if simType.optimisation
    simType.main = 1;
end

%% Load Data:

config;

%% Main Simulation:

if simType.main

    tspan = [0 10*data.orbit.period];
    options = data.ode.highTol;
    
    [T,Y,out] = integrateOdeFun(@odeFun, tspan, data.ode.Y0, options, data);

end

%% Failures ( no thrust and blockage of flow control valve):

if simType.failures
    
    failures
    
end

%% Linearization:

if simType.linearization
    
    linearization
    
end

%% Sensitivity Analysis:

if simType.sensitivity
    
    sensitivityAnalysis
    
end

%% Integration Analysis

if simType.integrationAnalysis
   
    integrationAnalysis
    
end

%% Optimization:

if simType.optimisation
    
    optimisation
    
    tspan = [0 10*data.orbit.period];
    options = data.ode.highTol;
    
    [TOpt,YOpt,outOpt] = integrateOdeFun(@odeFun, tspan, dataOpt.ode.Y0, options, dataOpt);
    
end

%% Plot Results:

plots;
