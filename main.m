% Modeling and Simulation of Aerospace Systems (2020/2021)
% Project
% Authors: Giulio Pacifici, Lorenzo Porcelli, Giacomo Velo

clear;clc;close all;

%% Load Data:

config;

%% Integration:

% Time span array:
tspan = [0 10*data.orbit.period];

% Integration:
options = odeset('AbsTol',1e-14,'RelTol',1e-13);

tic
[sol,T,Y,out] = integrateOdeFun(@odeFun, tspan, data.ode.Y0, options, data);
toc

%% Plots:

plots;

%% PSD of the residual acceleration:

% The solution is evaluated on an equispaced grid in order to use the FFT
% algorithm:
Nt = 1e5; %There are just a few peaks around 0, so the number of points can be quite low
TEq = linspace(0, T(end), Nt)';
dt = TEq(2) - TEq(1);
YEq = deval(sol, TEq)';

for k = 1:length(TEq)
    [~,outTemp] = odeFun(TEq(k),YEq(k,:)',data);
    thrustEq = outTemp.thrust;
    dragVEq = outTemp.dragV;
end

% Residual acceleration over time:
resAccEq = out.thrust + out.dragV;

% Definition of the frequency axis:
Nf = 2^ceil(log2(Nt));
fAxis = (-Nf/2:Nf/2-1)/Nf/dt; %Conversion to Hz

% FFT of the residual acceleration:
resAccF = fftshift(fft(resAccEq, Nf));
% No phase delay compensation is needed since the signal resAccEq is
% considered to start at 0

% PSD of the residual acceleration:
psdAcc = abs(resAccF).^2/(TEq(end));
plot(fAxis, psdAcc)
grid on