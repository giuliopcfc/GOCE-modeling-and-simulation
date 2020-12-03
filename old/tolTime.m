relTol = 10.^[-14:1:-6];
absTol = 10.^[-14:1:-6];
config

% Time span array:
tspan = [0 data.orbit.period];
T = zeros(length(absTol), length(relTol));

for i = 1:150
    i
    options = odeset('AbsTol',1e-10,'RelTol',1e-8);
    integrateOdeFun(@odeFun, tspan, data.ode.Y0, options, data);
end

tspan = [0 5*data.orbit.period];
for i = 1:length(absTol)
    i
    for j = 1:length(relTol)
        % Integration:
        options = odeset('AbsTol',absTol(i),'RelTol',relTol(j));
        tic
        integrateOdeFun(@odeFun, tspan, data.ode.Y0, options, data);
        T(i,j) = toc;
    end
end
%%
figure,
hold on
for i = 1:length(absTol)
    
    plot(relTol,T(i,:),'linewidth',1.5);
end
set(gca,'XScale','log')
legend('-14','-13','-12','-11','-10','-9','-8','-7','-6')
grid on
xlabel('Rel Tol'), ylabel('Time [s]')