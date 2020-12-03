% Optimizable variables: [massSpool, kPropFCV, kIntFCV, kI, kPropAcc, kDerAcc];
config;

ratiosVars = cell(6,1);

% Arrays of variation ratios:
ratiosVars{1} = [0.4 0.6 0.8 1.1];          n1 = length(ratiosVars{1});
ratiosVars{2} = [0.5 5 10 20 50];           n2 = length(ratiosVars{2});
ratiosVars{3} = [1 5 10 20 50 100 1000];    n3 = length(ratiosVars{3});
ratiosVars{4} = [1 5 10 20 50 100 1000];    n4 = length(ratiosVars{4});
ratiosVars{5} = [0.01 0.1 0.5 0.8 1];       n5 = length(ratiosVars{5});
ratiosVars{6} = [0.01 0.1 0.5 0.8 1];       n6 = length(ratiosVars{6});

N = n1 + n2 + n3 + n4 + n5 + n6;

ratioMat = ones(6,N);

ratioMat(1,1:n1) = ratiosVars{1};
ratioMat(2,n1+1:n1+n2) = ratiosVars{2};
ratioMat(3,n1+n2+1:n1+n2+n3) = ratiosVars{3};
ratioMat(4,n1+n2+n3+1:n1+n2+n3+n4) = ratiosVars{4};
ratioMat(5,n1+n2+n3+n4+1:n1+n2+n3+n4+n5) = ratiosVars{5};
ratioMat(6,n1+n2+n3+n4+n5+1:end) = ratiosVars{6};

% Perform Simulation:
ratiosJArr = zeros(N,1);

tspan = [0:1:data.orbit.period];
options = odeset('AbsTol',1e-8,'RelTol',1e-6);

% Nominal simulation:
J0 = costFun(tspan, data.ode.Y0, options, data);

for i = 1:N
    i/N*100
    dataI = data;
    dataI.FCV.massSpool = data.FCV.massSpool*ratioMat(1,i);
    dataI.FCV.kProp = data.FCV.kProp*ratioMat(2,i);
    dataI.FCV.kInt = data.FCV.kInt*ratioMat(3,i);
    dataI.FCV.kI = data.FCV.kI*ratioMat(4,i);
    dataI.accelerometer.kProp = data.accelerometer.kProp*ratioMat(5,i);
    dataI.accelerometer.kDer = data.accelerometer.kDer*ratioMat(6,i);
    
    J = costFun(tspan, dataI.ode.Y0, options, dataI);
    
    ratiosJArr(i) = J/J0;
    
end

%% Organize results:
ratiosJ = cell(6,1);

ratiosJ{1} = ratiosJArr(1:n1);
ratiosJ{2} = ratiosJArr(n1+1:n1+n2);
ratiosJ{3} = ratiosJArr(n1+n2+1:n1+n2+n3);
ratiosJ{4} = ratiosJArr(n1+n2+n3+1:n1+n2+n3+n4);
ratiosJ{5} = ratiosJArr(n1+n2+n3+n4+1:n1+n2+n3+n4+n5);
ratiosJ{6} = ratiosJArr(n1+n2+n3+n4+n5+1:end);

figure,
hold on
for i = 1:6
    
    plot(ratiosVars{i},ratiosJ{i},'-o','linewidth',1.5)
    
end
set(gca,'XScale','log','YScale','log')
legend('Spool Mass', 'Proportional Gain FCV', 'Integral Gain FCV', '$K_I$',...
    'Proportional Gain Acc.','Derivative Gain Acc.','interpreter','latex')
grid on, box on
xlabel('Percentage Variation'), ylabel('J/J0')

%% Functions:
function J = costFun(tspan, Y0, options, data)

[~,~,out] = integrateOdeFun(@odeFun, tspan, Y0, options, data);

J = norm(out.residualAcc);
end
