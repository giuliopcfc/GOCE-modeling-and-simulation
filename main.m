% Modeling and Simulation of Aerospace Systems (2020/2021)
% Project
% Authors: Giulio Pacifici, Lorenzo Porcelli, Giacomo Velo

clear;clc;close all;

%% Load Data:

config;

%% Generation of noise:

dtN = 0.1;
tNoise = [0:dtN:1000];
noise = rand(length(tNoise),1) - 0.5;
data.noise = @(t) interp1(tNoise, noise, mod(t, tNoise(end)), 'nearest');

%% Integration:

% Time span array:
dt = 0.5; % Time step [s]
tspan = [0:dt:5*data.orbit.period];

% Integration:
options = odeset('AbsTol',1e-14,'RelTol',1e-13);

tic
[T,Y,out] = integrateOdeFun(@odeFun, tspan, data.ode.Y0, options, data);
toc

%% Plots:

plotResults(T, Y, out);

%% PSD of the residual acceleration:

[psdResAcc, freq] = FFTPSD(out.residualAcc*1e6,1/dt);

psdResAcc = sqrt(psdResAcc);

figure,
loglog(freq, psdResAcc)
grid on

%% Noisy condition (accelerometer + thruster)

data.accelerometer.noiseSwitch = 1;
data.thruster.noiseSwitch = 1;

% Integration:
options = odeset('AbsTol',1e-8,'RelTol',1e-6);

tic
[TN,YN,outN] = integrateOdeFun(@odeFun, tspan, data.ode.Y0, options, data);
toc

plotResults(TN, YN, outN);

[psdResAccN, freqN] = FFTPSD(outN.residualAcc*1e6,1/dt);

psdResAccN = sqrt(psdResAccN);

figure,
loglog(freqN, psdResAccN)
grid on
