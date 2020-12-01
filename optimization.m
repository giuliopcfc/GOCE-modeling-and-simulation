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
tspan = [0: 10 : data.orbit.period];

odeOptions = odeset('AbsTol',1e-10,'RelTol',1e-8);

%% Optimization: 

% Initial guess:
x0 = [ data.accelerometer.kProp/10    
       data.accelerometer.kDer/10
       data.FCV.massSpool          
       data.FCV.kProp*10              
       data.FCV.kInt               
       data.FCV.kI*10             ];

% percentage values of lower and upper boundaries wrt the provided data
lbPct = [ 0.01
          0.01
          0.1
          1-1e-4  
          1-1e-4
          1-1e-4];
      
ubPct = [ 1+1e-4
          1+1e-4 
          1+1e-4     
          50 
          5
          50    ];

lb = lbPct.*x0;
ub = ubPct.*x0;

optOptions = optimset('Display','Iter','TolFun',1e-10);

x = fmincon(@(x) costFun(x, tspan, data.ode.Y0, odeOptions, data),x0,...
 [],[],[],[],lb,ub,[],optOptions);

%% Solution found:
dataOpt = data;

dataOpt.accelerometer.kProp    = x(1);
dataOpt.accelerometer.kDer     = x(2);
dataOpt.FCV.massSpool          = x(3);
dataOpt.FCV.kProp              = x(4);
dataOpt.FCV.kInt               = x(5);
dataOpt.FCV.kI                 = x(6);

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