function [T,Y,out] = integrateOdeFun(odeFun, tspan, Y0, options, data)
%
% Function to integrate the ode function of the system and retrieve other
% parameters.
%  
% INPUT:
%   odeFun          Ode function
%   tspan           Time span array
%   Y0              Initial conditions for state array
%   options         Options for ode15s integrator
%   data            Input data struct
% 
% OUTPUT:
%   T [nT,1]        Time array
%   Y [nT,nY]       State matrix
%   out             Output struct that contains the time evolution 
%                   of some parameters of the system 
%

[T,Y] = ode15s(odeFun,tspan,Y0,options,data);

out = struct();
out.aRes = T; out.aDragV = T;

for i = 1:length(T)
    
    [~,outTemp] = odeFun(T(i),Y(i,:)',data);
    
    out.aRes(i) = outTemp.aRes;
    out.aDragV(i) = outTemp.aDragV;
end

end