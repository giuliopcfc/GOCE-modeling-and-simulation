%% Failures:
% In this script the integration of the failures is carried out. Due to the
% presence of discontinuities, the integration is divided into five parts.

dataF = data;

% Set time span for no thrust case:
dataF.noThrust.tInitial = 1000;
dataF.noThrust.tFinal = 1000 + 2*dataF.orbit.period;

% Set time span for valve blockage case:
dataF.blockFCV.tInitial = 3.7e4;
dataF.blockFCV.tFinal = 3.7e4 + 2*dataF.orbit.period;

options = data.ode.highTol;

% 5 subsequent integrations:
tspan(1,:) = [0 dataF.noThrust.tInitial];
tspan(2,:) = [dataF.noThrust.tInitial dataF.noThrust.tFinal];
tspan(3,:) = [dataF.noThrust.tFinal dataF.blockFCV.tInitial];
tspan(4,:) = [dataF.blockFCV.tInitial dataF.blockFCV.tFinal];
tspan(5,:) = [dataF.blockFCV.tFinal 10*dataF.orbit.period];

Y0 = dataF.ode.Y0;
TF = []; YF = [];

for i = 1:5
    
    switch i
        case 2
            dataF.noThrust.switch = 1;
        case 3
            dataF.noThrust.switch = 0;
        case 4
            dataF.blockFCV.switch = 1;
        case 5
            dataF.blockFCV.switch = 0;
    end
    
    [TI,YI] = ode15s(@odeFun, tspan(i,:), Y0, options, dataF);
    
    if i ~= 1
        TI = TI(2:end);
        YI = YI(2:end,:);
    end
    
    TF = [TF; TI]; YF = [YF; YI];
    
    Y0 = YI(end,:)';
    
end

% the values of thrust and dragV are here retrieved:

outF = struct();
outF.thrust = TF; outF.dragV = TF; outF.VC = TF;

for i = 1:length(TF)
    
    dataF.noThrust.switch = 1;
    dataF.blockFCV.switch = 1;
    
    [~,outTemp] = odeFun(TF(i),YF(i,:)',dataF);
    
    outF.thrust(i) = outTemp.thrust;
    outF.dragV(i) = outTemp.dragV;
    outF.VC(i) = outTemp.VC;
    
end

outF.residualAcc = (outF.thrust + outF.dragV)/data.goce.mass;
