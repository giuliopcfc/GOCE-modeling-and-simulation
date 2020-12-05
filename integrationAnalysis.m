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

% Absolute tolerance and relative tolerance arrays:
relTol = 10.^[-13:1:-8];
absTol = 10.^[-13:1:-8];

% Integration options:
tspan = [0 data.orbit.period];
cpuTimes = zeros(length(absTol), length(relTol));

% Integration with different valuess of tolerance:
for i = 1:length(absTol)
    for j = 1:length(relTol)
        options = odeset('AbsTol',absTol(i),'RelTol',relTol(j));
        tic
        [T,Y] = ode15s(@odeFun,tspan,data.ode.Y0,options,data);
        cpuTimes(i,j) = toc;
    end
end

%%
figure,
hold on
legendAbsTol = cell(length(absTol),1);
for i = 1:length(absTol)
    plot(relTol,cpuTimes(i,:),'linewidth',1.5);
    
    legendAbsTol{i} = ['AbsTol $= 10^{',num2str(log10(absTol(i))),'}$'];
end
set(gca,'XScale','log')
legend(legendAbsTol,'interpreter','latex')
grid on
xlabel('RelTol'), ylabel('Time [s]')

%% Plot of the stability regions of BDFs along with eigenvalues from the linearization 

config;
linearization;

x = [0 0]; y = [-8 8];

n = 250;
t = linspace(0,2*pi,n);
z = exp(1i*t);

figure,
plot(8*y,x, 'k', 'HandleVisibility', 'off')
hold on
plot(x,8*y, 'k', 'HandleVisibility', 'off')

d = 1-1./z; r = 0;

for i = 1:5
  r = r+(d.^i)/i;
  plot(r, 'LineWidth', 2)
end

title('Backward differentiation formulas orders 1-5 (exteriors of curves)')
hold on
xlim([-20 20])
xlabel('Re\{h\lambda\}')
ylim([-15 15])
ylabel('Im\{h\lambda\}')
grid on
legend('BDF1','BDF2','BDF3','BDF4','BDF5', 'Location', 'best')

h = 2; %Mean value of the stepsize coming from the integrations

scatter(real(eigA), imag(eigA), 'b', 'filled', 'HandleVisibility', 'off');
hold on
scatter(h*real(eigA), h*imag(eigA), 'r', 'filled', 'HandleVisibility', 'off'); 