%% Integration Analysis:
% Analysis of the integration for the numerical model.

%% Choice of the integrator:

IntegrationTime = zeros(4,1);
Integrator = {'ode15s';'ode23s';'ode23t';'ode23tb'};

% Integration options:
tspan = [0 data.orbit.period];
options = odeset('AbsTol',1e-8,'RelTol',1e-8);

tic
[T,Y] = ode15s(@odeFun,tspan,data.ode.Y0,options,data);
IntegrationTime(1) = toc;
tic
[T,Y] = ode23s(@odeFun,tspan,data.ode.Y0,options,data);
IntegrationTime(2) = toc;
tic
[T,Y] = ode23t(@odeFun,tspan,data.ode.Y0,options,data);
IntegrationTime(3) = toc;
tic
[T,Y] = ode23tb(@odeFun,tspan,data.ode.Y0,options,data);
IntegrationTime(4) = toc;

% Display table:
table(Integrator,IntegrationTime)

%% Choice of the tolerance values:

nRuns = 100; % Number of runs

% Absolute tolerance and relative tolerance arrays:
relTol = 10.^[-13:1:-8];
absTol = 10.^[-14:1:-8];

% Integration options:
tspan = [0 data.orbit.period];
cpuTimes = zeros(length(absTol), length(relTol), nRuns);

% Integration with different valuess of tolerance:
for k = 1:nRuns
    for i = 1:length(absTol)
        for j = 1:length(relTol)
            options = odeset('AbsTol',absTol(i),'RelTol',relTol(j));
            tic
            [T,Y] = ode15s(@odeFun,tspan,data.ode.Y0,options,data);
            cpuTimes(i,j,k) = toc;
        end
    end
end

cpuTimes = mean(cpuTimes,3);