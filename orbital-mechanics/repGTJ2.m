function [a,aunpert] = repGTJ2(input)
% repGTJ2 Repeating groundtrack with J2 effect
% 
% Function to compute the required semimajor axis for a repeating ground
% track with k satellite revolutions and m Earth rotations taking into
% account the J2 effect.
% 
% PROTOTYPE:
%  a = repGTJ2(input)
% 
% INPUT:
%  input           Input structure
% 
% OUTPUT:
%  a [1]           Required semimajor axis of the orbit for perturbed problem [km]
%  aunpert [1]     Required semimajor axis of the orbit for unperturbed problem [km]
% 
% CONTRIBUTORS:
%  Lyle Campbell
%  Giulio Pacifici 
%  Luca Rizzieri 
%  Davide Sisana
% 
% VERSIONS:
%  02-11-2019: First version
%  15-12-2019: ( Giulio Pacifici ) Added aunpert to the outputs.
%

k_E = input.k_E;
R_E = astroConstants(23);
w_E = norm(input.w_E);
e = input.e0;
i = input.i0;
m = input.m;
k = input.k;

J2 = 0.00108263;

OMdot = @(x) -(3/2*sqrt(k_E)*J2*R_E^2/(1-e^2)^2/x^(7/2))*cos(i);
omdot = @(x) -(3/2*sqrt(k_E)*J2*R_E^2/(1-e^2)^2/x^(7/2))*(5/2*(sin(i))^2-2);
M0dot = @(x) -(3/2*sqrt(k_E)*J2*R_E^2/(1-e^2)^2/x^(7/2))*(1-3/2*(sin(i)^2));

n = @(x) sqrt(k_E/x^3);

f = @(x) m/k - (w_E - OMdot(x))/(n(x) + omdot(x) + M0dot(x));

aunpert = (k_E^(1/2)*m/w_E/k)^(2/3);

options = optimset('TolX',1e-14);
a = fzero(f,aunpert,options);

end