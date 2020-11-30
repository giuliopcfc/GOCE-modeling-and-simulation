% Modeling and Simulation of Aerospace Systems (2020/2021)
% Project
% Authors: Giulio Pacifici, Lorenzo Porcelli, Giacomo Velo

clear;clc;close all;

%% Load Data:

config;

%% Integration:

% Time span array:
dt = 0.5; % Time step [s]
tspan = [0:dt:10*data.orbit.period];

% Integration:
options = odeset('AbsTol',1e-14,'RelTol',1e-13);

tic
[T,Y,out] = integrateOdeFun(@odeFun, tspan, data.ode.Y0, options, data);
toc

%% Plots:

plots;

%% PSD of the residual acceleration:

[psdResAcc, freq] = FFTPSD(out.residualAcc*1e6,1/dt);

psdResAcc = sqrt(psdResAcc);

figure,
loglog(freq, psdResAcc)
grid on