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


% 

% thrust = ionThruster(xFCV,data);

thrust = 0;  % [N]

YGPE = Y;
dYGPE = GPE(YGPE,thrust,data); % RHS of Gauss Planetary Equations

dY = dYGPE;
end