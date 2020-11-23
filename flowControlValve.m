function  dYFCV = flowControlValve(YFCV,VOut,data)

% Load data:
kI = data.FCV.kI;
kProp = data.FCV.kProp; kIint = data.FCV.kInt;
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
i = kProp*VOut + kIint*intVOut;

x0 = 10*A0; % Rest position of the spring [m]

% EOM of the spool:
dx = v;

dv = 1/m*( kSpring*(x0 - x) - kI*i - c*v );

% BCS for the spool:
if (x <= 0 && dv < 0) || (x >= D0 && dv > 0)
    dv = 0;
end

% Derivatives of the state variables:
dYFCV = [VOut; dx; dv];

end