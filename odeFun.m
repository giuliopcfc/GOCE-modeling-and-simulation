function dY = odeFun(t,Y,data)
%
% Right Hand Side of the ODE system.
%  
% INPUT:
%  t        Time instant
%  Y        Keplerian elements array
%  data     data struct
% 
% OUTPUT:
%  dY       Time derivative of the state vector
% 

VOut = 0;

YFCV = Y(1:3);
dYFCV = flowControlValve(YFCV,VOut,data);

% thrust = ionThruster(YFCV(2),data);

thrust = 0;  % [N]

YGPE = Y(4:9);
dYGPE = GPE(YGPE,thrust,data); % RHS of Gauss Planetary Equations

dY = [dYFCV; dYGPE];
end