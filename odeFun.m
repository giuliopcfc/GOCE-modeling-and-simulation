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

YGPE = Y;
dYGPE = GPE(YGPE,data); % RHS of Gauss Planetary Equations

dY = dYGPE;
end