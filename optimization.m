%{
Optimization variables:

data.accelerometer.kProp    Controller Proportional Gain
data.accelerometer.kDer     Controller Derivative Gain
data.FCV.massSpool          Spool Mass [kg]
data.FCV.kProp              Controller Proportional Gain
data.FCV.kInt               Controller Integral Gain
data.FCV.kI                 Proportionality Coeffcient Current - Spool
%}

%% Integration:

% Time span array:
tspan = [0: 1 : data.orbit.period];

odeOptions = odeset('AbsTol',1e-12,'RelTol',1e-10);

%% Optimization: 

% Initial guess:
x0 = [ data.accelerometer.kProp 
       data.FCV.kInt                
       data.FCV.kI             ];

% percentage values of lower and upper boundaries wrt the provided data
lbPct = [ 0.5
          1
          1];
      
ubPct = [ 1
          2
          2];

lb = lbPct.*x0;
ub = ubPct.*x0;

optOptions = optimset('Display','Iter','TolFun',1e-5);

x = fmincon(@(x) costFun(x, tspan, data.ode.Y0, odeOptions, data),x0,...
 [],[],[],[],lb,ub,[],optOptions);

%% Solution found:
dataOpt = data;

dataOpt.accelerometer.kProp    = x(1);
dataOpt.FCV.kInt               = x(2);
dataOpt.FCV.kI                 = x(3);

%% Functions:
function J = costFun(x, tspan, Y0, odeOptions, data)

data.accelerometer.kProp    = x(1);
% data.accelerometer.kDer     = x(2);
% data.FCV.massSpool          = x(3);
% data.FCV.kProp              = x(4);
data.FCV.kInt               = x(2);
data.FCV.kI                 = x(3);

[~,~,out] = integrateOdeFun(@odeFun, tspan, Y0, odeOptions, data);

J = norm(abs(out.thrust + out.dragV));
end