function dY = odeCart(t,Y,input)
% odeCart Cartesian ode function
% 
% Input function for the ode integration of the two-body problem made by
% Cartesian state vector Y = [rrx;rry;rrz;vvx;vvy;vvz].
% 
% PROTOTYPE:
%  dY = odeCart(~,Y,input)
%  
% INPUT:
%  Y     [6,1]      Cartesian state vector at a certain time [km] and [km/s]
%  input [1]        Input parameters:
%                   - k_E  Gravity constant of Earth [km^3/s^2]
%                   - w_e  Earth's rotation velocity (eastwards) [deg/s]
%                   - Am   Ratio between Reference area and Spacecraft mass [m^2/kg]
%                   - cd   Drag coefficient [-]
% 
% OUTPUT:
%  dY    [6,1]      Time derivative of the state vector [km/s] and [km/s^2]
% 
% CONTRIBUTORS:
%  Lyle Campbell
%  Giulio Pacifici 
%  Luca Rizzieri 
%  Davide Sisana
% 
% VERSIONS:
%  02-11-2019: First version
%

k_E = input.k_E;
w_E = input.w_E;
Am = input.Am;
cd = input.cd;

% Initialize vectors:

rr = Y(1:3); vv = Y(4:6);
dY = zeros(6,1);
dY(1:3) = Y(4:6);                   % Position derivative
r = norm(rr);                       % Norm of position vector

% Get the perturbing acceleration:
ap = [0;0;0];

switch input.type
    case 0                           % unperturbed motion
        ap = [0;0;0];
    case 1                           % only secular J2 effects perturbation
        aJ2 = pertJ2(rr,k_E);
        ap = aJ2;
    case 2                           % only aerodynamic drag perturbation
        vrel = vv - cross(w_E,rr);
        aD = pertD(Am,cd,vrel,r);
        ap = aD;
    case 3                           % J2+Drag perturbation
        aJ2 = pertJ2(rr,k_E);
        vrel = vv - cross(w_E,rr);
        aD = pertD(Am,cd,vrel,r);
        ap = aJ2 + aD;     
end

dY(4:6) = -k_E/r^3*rr+ap;            % Velocity derivative


