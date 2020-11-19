function [t,kep,cart] = orbitIntegration(input)
% orbitIntegration
% 
% Integrates orbit given initial state array, method of integration and
% perturbation.
% 
% PROTOTYPE:
%  [t,kep,cart] = orbitIntegration(input)
%  
% INPUT:
%  input           Input structure in which the following fields must be
%                  specified:  
%                   method     Integration method, 'cart' for cartesian and 'GPE' for Gauss
%                              planetary equations
%                   type       Perturbation type, 0 for unperturbed, 1 for J2 only, 2 for drag
%                              only and 3 for J2 and drag.
% 
% OUTPUT:
%  t [N,1]         Integration time array [s]
%  kep [N,6]       Keplerian elements for each time step 
%  cart [N,6]      Cartesian elements (position and velocity vectors) for
%                  each time step
% CONTRIBUTORS:
%  Lyle Campbell
%  Giulio Pacifici 
%  Luca Rizzieri 
%  Davide Sisana
% 
% VERSIONS:
%  03-02-2019: First version
%

% Set ode options:
options = odeset('RelTol',1e-13,'AbsTol',1e-14);

% Select integration method:
if strcmp(input.method,'GPE')
    
    % Integration with GPE method:
    [t,kep] = ode113(@odeGPE,input.tspan,input.Y0GPE,options,input);
    
    % Initialize cart matrix:
    cart = kep;
     
    % Fill cart matrix with cartesian elements by converting keplerian
    % elements for each time step:
    for k = 1:length(t)
        
        [rr,vv] = par2car(kep(k,1),kep(k,2),kep(k,3),kep(k,4),...
            kep(k,5),kep(k,6),input.k_E);
        
        cart(k,:) = [rr',vv'];
        
    end
    
elseif strcmp(input.method,'cart')
    
    % Integration with cartesian method:
    [t,cart] = ode113(@odeCart,input.tspan,input.Y0Cart,options,input);
    
    % Initialize kep matrix:
    kep = cart;
    
    
    % Fill cart matrix with keplerian elements by converting cartesian
    % elements for each time step:
    for k = 1:length(t)
        
        rr = cart(k,1:3)';
        vv = cart(k,4:6)';
        
        [a,e,i,OM,om,th] = car2par(rr,vv,input.k_E);
        
        kep(k,:) = [a,e,i,OM,om,th];
        
    end
    
end
