%% Sensitivity Analysis:
% The sensitivity analysis is here performed. 
% Variables: CD, mass of GOCE,proof mass of the accelerometer, c,
%             mass of the spool, KI, Xenon temperature

sigmasFrac = [0.2, 0.1, 0.1, 0.2, 0.1, 0.1, 0.05]; 
rng(1)
% Simulation options:
tSA = [0:10:2*data.orbit.period]; nT = length(tSA);
options = data.ode.lowTol;

N = 1000; % Maximum number of integrations

aRes = nan(nT,N); meanArr = aRes; stdArr = aRes;
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
    
    % Spool mass:
    dataI.FCV.massSpool = data.FCV.massSpool*(1 + randn*sigmasFrac(5));
    
    % Acceleration voltage:
    dataI.FCV.kI = data.FCV.kI*(1 + randn*sigmasFrac(6));
    
    % Xenon temperature:
    dataI.thruster.T2 = data.thruster.T2*(1 + randn*sigmasFrac(7));
    
    [~,~,outI] = integrateOdeFun(@odeFun, tSA, dataI.ode.Y0, options, dataI);
    
    aRes(:,i) = outI.aRes;
    
    stdArr(:,i) = std(aRes(:,1:i),[],2);
    meanArr(:,i) = mean(aRes(:,1:i),2);
    
    % Stopping criterium:
    if  abs((meanArr(end,i) - meanEndOld)/meanEndOld) < 1e-5
        break;
    end
    meanEndOld = meanArr(end,i);
end

meanResp = meanArr(:,i);
stdResp = stdArr(:,i);
