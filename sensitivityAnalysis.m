
%% Non Optimisable: [CD, mass, massAcc, c, pXe, TXe, deltaV]
tic
sigmasFrac = [0.2, 0.1, 0.05, 0.2, 0.1, 0.1, 0.05];

% Simulation options:
tspan = [0:10:2*data.orbit.period]; nT = length(tspan);
options = odeset('AbsTol',1e-10,'RelTol',1e-8);

N = 1000; % Maximum number of integrations

residualAcc = nan(nT,N); meanArr = residualAcc; stdArr = residualAcc;
meanEndOld = 1000;

for i = 1:N
    
    dataI = data;
    
    % CD:
    CD = data.goce.mass/data.goce.balCoeff/data.goce.area*(1 + randn*sigmasFrac(1));
    
    % mass:
    dataI.goce.mass = data.goce.mass*(1 + randn*sigmasFrac(2));
    
    dataI.goce.balCoeff = dataI.goce.mass/dataI.goce.area/CD;
    
    % Accelerometer mass variation:
    dataI.accelerometer.mass = dataI.accelerometer.mass*(1 + randn*sigmasFrac(3));
    
    % Viscous force coefficient:
    dataI.FCV.c = dataI.FCV.c*(1 + randn*sigmasFrac(4));
    
    % Xenon pressure:
    dataI.thruster.p2 = data.thruster.p2*(1 + randn*sigmasFrac(5));
    
    % Xenon temperature:
    dataI.thruster.T2 = data.thruster.T2*(1 + randn*sigmasFrac(6));
    
    % Acceleration voltage:
    dataI.thruster.deltaV = data.thruster.deltaV*(1 + randn*sigmasFrac(7));
    
    [~,~,outI] = integrateOdeFun(@odeFun, tspan, dataI.ode.Y0, options, dataI);
    
    residualAcc(:,i) = outI.residualAcc;
    
    stdArr(:,i) = std(residualAcc(:,1:i),[],2);
    meanArr(:,i) = mean(residualAcc(:,1:i),2);
    
    % Stopping criterium:
    if  abs((meanArr(end,i) - meanEndOld)/meanEndOld) < 1e-5
        break;
    end
    meanEndOld = meanArr(end,i);
end

figure,
plot(tspan,meanArr(:,i))
hold on
plot(tspan,meanArr(:,i) + stdArr(:,i))
hold on
plot(tspan,meanArr(:,i) - stdArr(:,i))
grid on, box on
title('System Response Envelope Under Uncertainties')
legend('Mean Response','Mean Response $ +\, \sigma (t)$','Mean Response $ -\, \sigma (t)$',...
    'interpreter','latex')
xlabel('Time $[s]$'), ylabel('Residual Acceleration $[m/s^2]$')

%% Optimizable variables: [massSpool, kPropFCV, kIntFCV, kI, kPropAcc, kDerAcc];

% Arrays of variation ratios:
ratiosVars = cell(6,1);  
ratiosVars{1} = [0.5:0.4:2];  ratiosVars{2} = [0.5:0.35:2]; 
ratiosVars{3} = [0.5:0.45:2]; ratiosVars{4} = [0.55,1,1.55,1.95];
ratiosVars{5} = [0.5:0.5:2];  ratiosVars{6} = [0.5:0.5:2]; 

% Simulation Options:
tspan = [0:1:data.orbit.period];
options = odeset('AbsTol',1e-10,'RelTol',1e-8);

% Nominal simulation:
J0 = costFun(tspan, data.ode.Y0, options, data);

% Organize results:
ratiosJ = cell(6,1);

for iVar = 1:6
    
    ratiosJ{iVar} = zeros(length(ratiosVars{iVar}),1);
    
    for i = 1:length(ratiosVars{iVar})
        dataI = data;
        
        switch iVar
            case 1, dataI.FCV.massSpool = data.FCV.massSpool*ratiosVars{iVar}(i);
            case 2, dataI.FCV.kProp = data.FCV.kProp*ratiosVars{iVar}(i);
            case 3, dataI.FCV.kInt = data.FCV.kInt*(ratiosVars{iVar}(i));
            case 4, dataI.FCV.kI = data.FCV.kI*ratiosVars{iVar}(i);
            case 5, dataI.accelerometer.kProp = data.accelerometer.kProp*ratiosVars{iVar}(i);
            case 6, dataI.accelerometer.kDer = data.accelerometer.kDer*ratiosVars{iVar}(i);
        end
        
        if ratiosVars{iVar}(i) == 1
            ratiosJ{iVar}(i) = 1;
        else
            J = costFun(tspan, dataI.ode.Y0, options, dataI);
            ratiosJ{iVar}(i) = J/J0;
        end
    end
    
end

% plots:
figure,
hold on
for i = 1:6
    
        plot(ratiosVars{i},ratiosJ{i},'-o','linewidth',1.5)

end
set(gca,'XScale','log')
legend('$m_{spool}$', '$k_{p,fcv}$', '$k_{i,fcv}$', '$K_I$',...
    '$k_{p,a}$','$k_{d,a}$','interpreter','latex')
grid on, box on
xlabel('Var/Var0'), ylabel('J/J0')
title('Variation of the cost function VS Variation of the optimisable variables')

%% Functions:
function J = costFun(tspan, Y0, options, data)

[~,~,out] = integrateOdeFun(@odeFun, tspan, Y0, options, data);

J = norm(out.residualAcc);
end
