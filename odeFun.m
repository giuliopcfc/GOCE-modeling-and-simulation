function dY = odeFun(t,Y,data)
%
% Right Hand Side of the ODE system.
%  
% INPUT:
%  t        Time instant
%  Y        State array
%  data     data struct
% 
% OUTPUT:
%  dY       Time derivative of the state array
% 
% NOTES:
%  Y(1:3) = YFCV = [intVOut; xFCV; vFCV]
%  Y(4:6) = YA = [xMass; vMass; VOut]
%  Y(7:12) = YGPE = [a; e; i; OM; om; theta]
% 

% Load data:
MU = data.const.MU_EARTH;

% Load state arrays:
YFCV = Y(1:3); YA = Y(4:6); YGPE = Y(7:12);

% Flow Control Valve:
dYFCV = flowControlValve(YFCV,YA(3),data);

% Ion Thruster:
thrust = ionThruster(YFCV(2),data);

% Initialize keplerian elements:
a = YGPE(1); e = YGPE(2); i = YGPE(3);
OM = YGPE(4); om = YGPE(5); f = YGPE(6);

% Passage from orbital parameters to cartesian coordinates:
[rr,vv] = kep2car(a,e,i,OM,om,f,MU);  

% Aerodynamic Drag:
aDrag = pertDrag(rr,vv,data);
% Drag component in the direction of velocity:
dragV = data.goce.mass*dot(aDrag,vv/norm(vv));

% Accelerometer:
dYA = accelerometer(YA, thrust, dragV, data);

% Orbital Mechanics:
dYGPE = GPE(YGPE,rr,vv,aDrag,thrust,data);

dY = [dYFCV; dYA; dYGPE];
a= 2;

end