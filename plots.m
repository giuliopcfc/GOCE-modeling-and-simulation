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

figure
plot(T,out.dragV)
title('DragV')

figure
plot(T, out.dragV + out.thrust)
title('Thrust + DragV')

figure,
plot(T,Y(:,1))
title('Int VOut')

figure,
plot(T,Y(:,6))
title('VOut')

figure,
plot(T,out.VC)
title('VC')
 
