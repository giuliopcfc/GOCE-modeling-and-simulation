set(0,'defaultTextInterpreter','latex');

figure,
plot(T,Y(:,4))
xlabel('Time [s]'), ylabel('Semi Major Axis [km]');
grid 

figure,
plot(T,Y(:,2)*1000)
xlabel('Time [s]'), ylabel('Spool Postion [mm]');
grid 