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
%  Y(1:3) = YFCV = [IVOut; xV; vV]
%  Y(4:6) = YA = [xA; vA; VOut]
%  Y(7:12) = YGPE = [a; e; i; OM; om; theta]
% 

% Load data:
MU = data.const.MU_EARTH;

% Load state arrays:
YFCV = Y(1:3); YA = Y(4:6); YGPE = Y(7:12);

% Load state variables:
xV = YFCV(2);   
VOut = YA(3);   
a = YGPE(1);        e = YGPE(2);    i = YGPE(3);
OM = YGPE(4);       om = YGPE(5);   f = YGPE(6);

% Flow Control Valve:
dYFCV = flowControlValve(YFCV,VOut,data);

% Off-Nominal Condition: Blockage of flow control valve:
if data.blockFCV.switch 
    if t >= data.blockFCV.tInitial && t <= data.blockFCV.tFinal
        dYFCV(2:3) = 0;
    end
end

% Ion Thruster:
[aThrust] = ionThruster(xV,data);

% Off-Nominal Condition: No Thrust:
if data.noThrust.switch 
    if t >= data.noThrust.tInitial && t <= data.noThrust.tFinal
        aThrust = 0;
    end
end

% From orbital parameters to cartesian coordinates:
[rr,vv] = kep2car(a,e,i,OM,om,f,MU);  

% Orbital Perturbations:
[aDrag, aJ2] = orbitalPerturbations(rr,vv,data);

aDragV = dot(aDrag,vv/norm(vv)); % Drag acceleration in velocity direction

% Accelerometer:
[dYA] = accelerometer(YA, aThrust, aDragV, data);

% Orbital Mechanics:
dYGPE = GPE(YGPE,rr,vv,aDrag,aJ2,aThrust,data);

dY = [dYFCV; dYA; dYGPE];

% Output parameters:
out = struct();
out.aDragV = aDragV;
out.aRes = aDragV + aThrust; % Residual acceleration

end