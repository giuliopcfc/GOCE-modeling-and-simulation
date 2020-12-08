% Modeling and Simulation of Aerospace Systems (2020/2021)
% Project
% Authors: Giulio Pacifici, Lorenzo Porcelli, Giacomo Velo
clear;clc;close all;
set(0,'defaultTextInterpreter','latex','defaultAxesFontSize',15);
set(0,'defaultAxesTickLabelInterpreter','latex');
set(0, 'defaultLegendInterpreter','latex');

%% Select simulation type:

simType.main = 1; % Main simulation                     (ETA: 3 seconds)

simType.failures = 1; % Simulation with failures        (ETA: 4 seconds)

simType.linearization = 1; % Linearization              (ETA: 10 seconds)

simType.sensitivity = 1; % Sensitivity Analysis         (ETA: 28 seconds)

simType.integrationAnalysis = 1; % Integration Analysis (ETA: 25 seconds)

simType.optimization = 1; % Optimization                (ETA: 120 seconds)

%% Check on linked simulation types:

if simType.integrationAnalysis
    simType.linearization = 1;
end
if simType.optimization
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

%% Failures (no thrust and blockage of flow control valve):

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

if simType.optimization
    
    optimization
    
end

%% Plot Results:

plots;
