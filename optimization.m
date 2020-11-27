%{
Optimization variables:

data.accelerometer.kProp    Controller Proportional Gain
data.accelerometer.kDer     Controller Derivative Gain
data.FCV.massSpool          Spool Mass [kg]
data.FCV.kProp              Controller Proportional Gain
data.FCV.kInt               Controller Integral Gain
data.FCV.kI                 Proportionality Coeffcient Current - Spool
%}

config; 

%% Integrate:

%%% Initial conditions:
D0 = sqrt(4*data.FCV.A0/pi);
% Initial State for flow control valve:
Y0FCV = [0; D0; 0];

% Initial State for accelerometer:
Y0A = [0; 0; 0];

% Orbital Mechanics:
a0 = data.orbit.altitude + data.const.R_MEAN; % Initial Semi Major Axis [km] %%%% R_MEAN o R_EQUATORIAL?? %%%%

% Initial State for GPEs:
Y0GPE = [a0; data.orbit.eccentricity; data.orbit.inclination*pi/180;...
        data.orbit.argPerigee*pi/180; data.orbit.RAAN*pi/180; data.orbit.theta0*pi/180]; 
    
% Assembly initial state array:
Y0 = [Y0FCV; Y0A; Y0GPE; 0];

% Time span array:
tspan = [0:50:pi*sqrt(a0^3/data.const.MU_EARTH)];

odeOptions = odeset('AbsTol',1e-10,'RelTol',1e-8);

%% Optimization: 

% Initial guess:
x0 = [ data.accelerometer.kProp    
       data.accelerometer.kDer
       data.FCV.massSpool          
       data.FCV.kProp              
       data.FCV.kInt               
       data.FCV.kI               ];            

lb = 0*x0';
ub = [data.accelerometer.kProp+1 data.accelerometer.kDer+1 3e-1 1e10 1e10 1e10]';

options = optimset('Display','Iter','TolFun',1e-5);

x = fmincon(@(x) costFun(x, tspan, Y0, odeOptions, data),x0,...
 [],[],[],[],lb,ub,[],options);

%% Solution found:
data.accelerometer.kProp    = x(1);
data.accelerometer.kDer     = x(2);
data.FCV.massSpool          = x(3);
data.FCV.kProp              = x(4);
data.FCV.kInt               = x(5);
data.FCV.kI                 = x(6);

odeOptions = odeset('AbsTol',1e-14,'RelTol',1e-13);
tspan = [0 2*2*pi*sqrt(a0^3/data.const.MU_EARTH)];
[T,Y,out] = integrateOdeFun(@odeFun, tspan, Y0, odeOptions, data);

plots

%% Functions:
function J = costFun(x, tspan, Y0, odeOptions, data)

data.accelerometer.kProp    = x(1);
data.accelerometer.kDer     = x(2);
data.FCV.massSpool          = x(3);
data.FCV.kProp              = x(4);
data.FCV.kInt               = x(5);
data.FCV.kI                 = x(6);

[~,~,out] = integrateOdeFun(@odeFun, tspan, Y0, odeOptions, data);

J = norm(abs(out.thrust + out.dragV));
end