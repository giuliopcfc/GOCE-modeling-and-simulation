function [dY,out] = odeFun(t,Y,data)
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
%  out      Output struct
% 
% NOTES:
%  Y(1:3) = YFCV = [intVOut; xFCV; vFCV]
%  Y(4:6) = YA = [xMass; vMass; VOut]
%  Y(7:12) = YGPE = [a; e; i; OM; om; theta]
%  Y(13) = Propellant mass consumed
% 

% Load data:
MU = data.const.MU_EARTH;

% Load state arrays:
YFCV = Y(1:3); YA = Y(4:6); YGPE = Y(7:12);

% Flow Control Valve:
dYFCV = flowControlValve(t,YFCV,YA(3),data);

% Ion Thruster:
[thrust, mDot] = ionThruster(YFCV(2),data);

if data.accelerometer.noiseSwitch || data.thruster.noiseSwitch ...
        || (data.accelerometer.noiseSwitch && data.thruster.noiseSwitch)
    noise = data.noise(t);
else
    noise = 0;
end

thrust = thrust + data.thruster.noiseSwitch*1e-4*noise;

% Off-nominal condition:
if data.noThrust.switch 
    if t >= data.noThrust.tInitial && t <= data.noThrust.tFinal
        thrust = 0;
    end
end

% Initialize keplerian elements:
a = YGPE(1); e = YGPE(2); i = YGPE(3);
OM = YGPE(4); om = YGPE(5); f = YGPE(6);

% From orbital parameters to cartesian coordinates:
[rr,vv] = kep2car(a,e,i,OM,om,f,MU);  

% Aerodynamic Drag:
aDrag = pertDrag(rr,vv,data);
% Drag component in the direction of velocity:
dragV = data.goce.mass*dot(aDrag,vv/norm(vv));

% Accelerometer:
[dYA,VC] = accelerometer(YA, thrust, dragV, data);

dYA(2) = dYA(2) + data.accelerometer.noiseSwitch*1e-9*noise;

% Orbital Mechanics:
dYGPE = GPE(YGPE,rr,vv,aDrag,thrust,data);

dY = [dYFCV; dYA; dYGPE; mDot];

% Output parameters:
out = struct();
out.thrust = thrust;
out.dragV  = dragV;
out.VC     = VC;

end