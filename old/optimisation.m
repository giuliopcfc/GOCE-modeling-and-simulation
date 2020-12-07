%{
Optimization variables:

data.accelerometer.kProp    Controller Proportional Gain
data.accelerometer.kDer     Controller Derivative Gain
data.FCV.massSpool          Spool Mass [kg]
data.FCV.kProp              Controller Proportional Gain
data.FCV.kInt               Controller Integral Gain
data.FCV.kI                 Proportionality Coeffcient Current - Spool
%}
%% Integration Settings:
tspan = [0: 1 : data.orbit.period];
odeOptions = data.ode.lowTol;
%% Nominal Values of the variables:

xNominal = [    data.accelerometer.kProp;    
                data.accelerometer.kDer;     
                data.FCV.massSpool;         
                data.FCV.kProp;              
                data.FCV.kInt;               
                data.FCV.kI;    ];            
            
%% Boundaries for the optimisation variables:
LB = xNominal - [3e5; 5e3; 0.15; 0.01; 0.1; 0.05];
UB = xNominal + [3e5; 5e3; 0.4; 10; 30; 30];

% Evaluation of the cost function at xNominal:
JNominal = costFun(xNominal, tspan, data.ode.Y0, odeOptions, data, 1); 

% Evaluation of the cost function at the boundaries:
JB = zeros(6,2);

for i = 1:6
    
    x = xNominal; x(i) = LB(i);
    JLB = costFun(x, tspan, data.ode.Y0, odeOptions, data, 1);
    
    x = xNominal; x(i) = UB(i);
    JUB = costFun(x, tspan, data.ode.Y0, odeOptions, data, 1);  
    
    JB(i,:) = [JLB JUB]/JNominal;
    
end

JB
%% Optimisation of data.accelerometer.kProp,data.FCV.kInt,data.FCV.kI:

x0 = xNominal([1 5 6]);
                
lb = LB([1 5 6])./x0; ub = UB([1 5 6])./x0;

optOptions = optimset('Display','Iter','TolFun',1e-7);

x1 = fmincon(@(x) costFun(x, tspan, data.ode.Y0, odeOptions, data, 2),...
 [1;1;1],[],[],[],[],lb,ub,[],optOptions);

%% Optimisation of data.accelerometer.kDer, data.FCV.massSpool, data.FCV.kProp:

x0 = xNominal([2 3 4]);

dataOpt1 = data;
dataOpt1.accelerometer.kProp = x1(1)*data.accelerometer.kProp;
dataOpt1.FCV.kInt            = x1(2)*data.FCV.kInt;
dataOpt1.FCV.kI              = x1(3)*data.FCV.kI;

lb = LB([2 3 4])./x0; ub = UB([2 3 4])./x0;

optOptions = optimset('Display','Iter','TolFun',1e-7);

x2 = fmincon(@(x) costFun(x, tspan, data.ode.Y0, odeOptions, dataOpt1, 3),...
 [1;1;1],[],[],[],[],lb,ub,[],optOptions);

%% Final Data struct:

dataOpt2 = dataOpt1;
dataOpt2.accelerometer.kDer     = x2(1)*data.accelerometer.kDer;
dataOpt2.FCV.massSpool          = x2(2)*data.FCV.massSpool;
dataOpt2.FCV.kProp              = x2(3)*data.FCV.kProp;

%% Plots:
options = data.ode.highTol;
tspan = [0 10*data.orbit.period];
[T,Y,out] = integrateOdeFun(@odeFun, tspan, data.ode.Y0, options, data);
[TOpt1,YOpt1,outOpt1] = integrateOdeFun(@odeFun, tspan, dataOpt1.ode.Y0, options, dataOpt1);
[TOpt2,YOpt2,outOpt2] = integrateOdeFun(@odeFun, tspan, dataOpt2.ode.Y0, options, dataOpt2);


figure,
plot(T,out.residualAcc,TOpt1,outOpt1.residualAcc,TOpt2,outOpt2.residualAcc)
legend('Nominal','Opt1','Opt2')
%% Functions:
function J = costFun(x, tspan, Y0, odeOptions, data, type)

switch type
    case 1
        data.accelerometer.kProp    = x(1);
        data.accelerometer.kDer     = x(2);
        data.FCV.massSpool          = x(3);
        data.FCV.kProp              = x(4);
        data.FCV.kInt               = x(5);
        data.FCV.kI                 = x(6);
    case 2 
        data.accelerometer.kProp = x(1)*data.accelerometer.kProp;
        data.FCV.kInt            = x(2)*data.FCV.kInt;
        data.FCV.kI              = x(3)*data.FCV.kI;
    case 3
        data.accelerometer.kDer     = x(1)*data.accelerometer.kDer;
        data.FCV.massSpool          = x(2)*data.FCV.massSpool;
        data.FCV.kProp              = x(3)*data.FCV.kProp;
end

[~,~,out] = integrateOdeFun(@odeFun, tspan, Y0, odeOptions, data);

J = norm(out.residualAcc);

end