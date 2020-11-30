function plotResults(T, Y, out)

set(0,'defaultTextInterpreter','latex');

figure,
plot(T,Y(:,7))
xlabel('Time [s]'), ylabel('Semi Major Axis [km]');
grid 

figure,
plot(T,Y(:,2)*1000)
xlabel('Time [s]'), ylabel('Spool Postion [mm]');
grid 

figure,
plot(T,out.thrust)
title('Thrust')
grid

figure
plot(T,out.dragV)
title('DragV')
grid

figure
plot(T, out.dragV + out.thrust)
title('Thrust + DragV')
grid

figure,
plot(T,Y(:,4))
title('x Acc')
grid

figure,
plot(T,Y(:,6))
title('VOut')
grid

end