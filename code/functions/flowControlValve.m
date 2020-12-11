function  dYFCV = flowControlValve(YFCV,VOut,data)
%
% Flow control valve (and PI controller) block.
%  
% INPUT:
%  YFCV [3,1]       State array
%  VOut             Output voltage of the read-out circuit
%  data             data struct
% 
% OUTPUT:
%  dYFCV [3,1]      Time derivative of the state array
%

% Load data:
kI = data.FCV.kI;
kProp = data.FCV.kProp; kInt = data.FCV.kInt;
m = data.FCV.massSpool; kSpring = data.FCV.kSpring; c = data.FCV.c;
A0 = data.FCV.A0;

% Load state variables:
IVOut = YFCV(1); xV = YFCV(2); vV = YFCV(3);

% BCS for the spool:
% x = 0 when the valve is completely open, x must be between 0 and D0.
D0 = sqrt(4*A0/pi); % Diameter of the orifice
if xV <= 0
    xV = 0;
    if vV < 0
        vV = 0;
    end
end
if xV >= D0
    xV = D0;
    if vV > 0
        vV = 0;
    end
end

% PI controller:
I = kProp*VOut + kInt*IVOut;

x0 = data.FCV.x0; % Rest position of the spring [m]

% EOM of the spool:
dxV = vV;

dvV = 1/m*( kSpring*(x0 - xV) - kI*I - c*vV );

% BCS for the spool:
if (xV <= 0 && dvV < 0) || (xV >= D0 && dvV > 0)
    dvV = 0;
end
    
% Derivatives of the state variables:
dYFCV = [VOut; dxV; dvV];

end