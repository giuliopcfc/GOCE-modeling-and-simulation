% Modeling and Simulation of Aerospace Systems (2020/2021)
% Project
% Authors: Giulio Pacifici, Lorenzo Porcelli, Giacomo Velo
tic
clear;clc;close all;
set(0,'defaultTextInterpreter','latex');

%% Simulation type:

simType.main = 1; % Main simulation

simType.noise = 1; % Simulation with noise

simType.failures = 1; % Simulation with failures

simType.orbitDecay = 1; % Simulation with orbit decay

simType.sensitivity = 1; % Sensitivity Analysis

simType.optimization = 1; % Optimization

simType.linearization = 1; % Linearization

%% Load Data:

config;

%% Main Simulation:

if simType.main
    
    % Time span array:
    tspan = [0 10*data.orbit.period];
    % Integration:
    options = odeset('AbsTol',1e-10,'RelTol',1e-8);
    
    [T,Y,out] = integrateOdeFun(@odeFun, tspan, data.ode.Y0, options, data);
    
    plotResults(T, Y, out);
    
end

%% Noisy condition (accelerometer + thruster):

if simType.noise
    
    dataN = data;
    
    dataN.accelerometer.noiseSwitch = 1;
    dataN.thruster.noiseSwitch = 1;
    dataN.accelerometer.noiseMagn = 1e-7;
    dataN.thruster.noiseMagn = 1e-4;
    
    % Time span array:
    tspan = [0 dataN.orbit.period];
    
    % Integration:
    options = odeset('AbsTol',1e-8,'RelTol',1e-6);
    
    [TN,YN,outN] = integrateOdeFun(@odeFun, tspan, dataN.ode.Y0, options, dataN);
    
    
    plotResults(TN, YN, outN);
    
end

%% Failures ( no thrust and blockage of flow control valve):

if simType.failures
    
    dataF = data;
    
    dataF.noThrust.switch = 1;
    dataF.noThrust.tInitial = 1000;
    dataF.noThrust.tFinal = 1000 + 2*dataF.orbit.period;
    
    dataF.blockFCV.switch = 1;
    dataF.blockFCV.tInitial = 3e4;
    dataF.blockFCV.tFinal = 3e4 + 2*dataF.orbit.period;
    
    % Time span array:
    tspan = [0 10*dataF.orbit.period];
    
    % Integration:
    options = odeset('AbsTol',1e-10,'RelTol',1e-8);
    
    [TF,YF,outF] = integrateOdeFun(@odeFun, tspan, dataF.ode.Y0, options, dataF);
    
    
    plotResults(TF, YF, outF);
    
end

%% Orbit Decay:

if simType.orbitDecay
    
    dataOD = data;
    
    dataOD.noThrust.switch = 1;
    dataOD.noThrust.tFinal = Inf;
    
    % Time span array:
    tspan = [0 1380*dataOD.orbit.period];
    
    % Integration:
    options = odeset('AbsTol',1e-10,'RelTol',1e-8);
    
    [TOD,YOD,outOD] = integrateOdeFun(@odeFun, tspan, dataOD.ode.Y0, options, dataOD);
    
    
    plotResults(TOD, YOD, outOD);
    
end

%% Linearization:

if simType.linearization
    
    linearization
    
end

%% Sensitivity Analysis:

if simType.sensitivity
    
    sensitivityAnalysis
    
end

%% Optimization:

if simType.optimization
    
    optimization
    
    options = odeset('AbsTol',1e-10,'RelTol',1e-8);
    tspan = [0 10*dataOpt.orbit.period];
    
    [TOpt,YOpt,outOpt] = integrateOdeFun(@odeFun, tspan, dataOpt.ode.Y0, options, dataOpt);
    
    %plotResults(TOpt, YOpt, outOpt);
    
end
toc