%% Optimization:
% The optimization of the system is here performed. The cost function is
% selected as the residual acceleration.
%{
Optimization variables:

data.accelerometer.kProp    Controller Proportional Gain
data.accelerometer.kDer     Controller Derivative Gain
data.FCV.kProp              Controller Proportional Gain
data.FCV.kInt               Controller Integral Gain
%}
%% Integration Settings:
tspan = [0: 1 : data.orbit.period];
odeOptions = data.ode.lowTol;

%% Nominal Values of the variables:

xNominal = [    data.accelerometer.kProp;
    data.accelerometer.kDer;
    data.FCV.kProp;
    data.FCV.kInt;               ];

%% Set boundaries:
LB = (1 - 0.5)*xNominal;
UB = (1 + 1)*xNominal;

% Normalize boundaries:
lb = LB./xNominal;
ub = UB./xNominal;

%% Optimization:

x0 = xNominal./xNominal; % Normalized initial guess

optOptions = optimset('Display','Iter','TolFun',1e-7);

x = fmincon(@(x) costFun(x, tspan, data.ode.Y0, odeOptions, data),...
    x0,[],[],[],[],lb,ub,[],optOptions);

%% Retrive optimized data struct:
dataOpt = data;
dataOpt.accelerometer.kProp    = x(1)*data.accelerometer.kProp;
dataOpt.accelerometer.kDer     = x(2)*data.accelerometer.kDer;
dataOpt.FCV.kProp              = x(3)*data.FCV.kProp;
dataOpt.FCV.kInt               = x(4)*data.FCV.kInt;

%% Simulation:
tspan = [0 2*data.orbit.period];
options = data.ode.highTol;

[TOpt,YOpt,outOpt] = integrateOdeFun(@odeFun, tspan, dataOpt.ode.Y0, options, dataOpt);

%% Functions:
function J = costFun(x, tspan, Y0, odeOptions, data)

data.accelerometer.kProp    = x(1)*data.accelerometer.kProp;
data.accelerometer.kDer     = x(2)*data.accelerometer.kDer;
data.FCV.kProp              = x(3)*data.FCV.kProp;
data.FCV.kInt               = x(4)*data.FCV.kInt;

[~,~,out] = integrateOdeFun(@odeFun, tspan, Y0, odeOptions, data);

J = norm(out.aRes);

end