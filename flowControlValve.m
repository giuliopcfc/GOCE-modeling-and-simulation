function  dYFCV = flowControlValve(t,YFCV,VOut,data)
%
% Right Hand Side of the ODE system.
%  
% INPUT:
%  t                Time instant
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
intVOut = YFCV(1); x = YFCV(2); v = YFCV(3);

% BCS for the spool:
% x = 0 when the valve is completely open, x must be between 0 and D0.
D0 = sqrt(4*A0/pi); % Diameter of the orifice
if x <= 0
    x = 0;
    if v < 0
        v = 0;
    end
end
if x >= D0
    x = D0;
    if v > 0
        v = 0;
    end
end

% PI controller:
i = kProp*VOut + kInt*intVOut;

x0 = data.FCV.x0; % Rest position of the spring [m]

% EOM of the spool:
dx = v;

dv = 1/m*( kSpring*(x0 - x) - kI*i - c*v );

% BCS for the spool:
if (x <= 0 && dv < 0) || (x >= D0 && dv > 0)
    dv = 0;
end

% Off-nominal condition:
if data.blockFCV.switch 
    if t >= data.blockFCV.tInitial && t <= data.blockFCV.tFinal
        dx = 0;
        dv = 0;
    end
end
    

% Derivatives of the state variables:
dYFCV = [VOut; dx; dv];

end