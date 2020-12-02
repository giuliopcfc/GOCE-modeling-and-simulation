data = struct();

%% Project data:

% GOCE:
data.goce.mass = 300; % GOCE mass [kg]
data.goce.area = 1.1; % GOCE frontal section [m^2]
data.goce.balCoeff = 300; % GOCE ballistic coefficient

% Accelerometer:
data.accelerometer.eps = 8.85e-12; % Accelerometer permittivity [F/m]
data.accelerometer.areaMass = 1.6e-3; % Accelerometer Seismic Mass Section [m^2]
data.accelerometer.mass = 0.32; % Accelerometer Seismic Mass [kg]
data.accelerometer.gap = 5e-4; % Electrodes-Mass Gap [m]
data.accelerometer.VBias = 10; % Accelerometer Bias Voltage [V]
data.accelerometer.C = 2e-12; % Capacitance [F]
data.accelerometer.kProp = 1e6; % Controller Proportional Gain
data.accelerometer.kDer = 5e4; % Controller Derivative Gain

% Flow Control Valve:
data.FCV.A0 = 1e-5; % Orifice Area [m^2]
data.FCV.massSpool = 2e-1; % Spool Mass [kg]
data.FCV.kProp = 0.1; % Controller Proportional Gain
data.FCV.kInt = 3; % Controller Integral Gain
data.FCV.kI = 0.2; % Proportionality Coeffcient Current - Spool
data.FCV.kSpring = 7e3; % Spring Coefficient [N/m]
data.FCV.c = 30; % Friction Coefficient [Ns/m]

% Ion Thruster:
data.thruster.T2 = 240; % Xenon Working Temperature [K]
data.thruster.p2 = 2e5; % Xenon Working Pressure [Pa]
data.thruster.mmXe = 131.293; % Xenon Molar Mass [kg/kmol]
data.thruster.kXe = 1.66; % Xenon Specific Heat Ratio
data.thruster.massIon = 2.188e-25; % Xenon Ion Mass [kg]
data.thruster.e = 1.6e-19; % Electron Charge [C]
data.thruster.deltaV = 2e3; % Acceleration Grid Voltage [V]

% Orbit:
data.orbit.altitude = 254.9; % Altitude [km]
data.orbit.inclination = 90; % Inclination [deg]
data.orbit.eccentricity = 0.0045; % Eccentricity 
data.orbit.argPerigee = 0; % Argument Of Perigee [deg]
data.orbit.RAAN = 0; % Right Ascention Of The Ascending Node [deg]
data.orbit.theta0 = 0; % Initial True Anomaly [deg]

% Constants:
data.const.MU_EARTH = 398600; % Earth's Gravitational Parameter [km^3/s^2]
data.const.W_EARTH = (2*pi + 2*pi/365.26)/24/3600; % Earth's rotational speed [rad/s]
data.const.R_EQUATORIAL = 6378.16; % Earth's equatorial radius [km]
data.const.R_POLAR = 6356.778; % Earth's polar radius [km]
data.const.R_MEAN = 6371.0088; % Earth's mean radius [km] (from https://en.wikipedia.org/wiki/Earth_radius#Mean_radius)
data.const.J2 = 0.00108263; % J2 factor
data.const.Ru = 8314.462; % Universal Gas Constant [J/kmol/K]


%% Off-nominal conditions:

data.noThrust.switch = 0;
data.noThrust.tInitial = 0;
data.noThrust.tFinal = 0;

data.blockFCV.switch = 0;
data.blockFCV.tInitial = 0;
data.blockFCV.tFinal = 0;

data.thruster.noiseSwitch = 0;
data.thruster.noiseMagn = 0;
data.accelerometer.noiseSwitch = 0;
data.accelerometer.noiseMagn = 0;

%% Compute additional data:

data.FCV.D0 = sqrt(4*data.FCV.A0/pi); % Diameter of the valve's orifice [m^2]

% Semi Major Axis [km]:
data.orbit.SMAxis = data.orbit.altitude + data.const.R_EQUATORIAL; 

% Orbit period:
data.orbit.period = 2*pi*sqrt(data.orbit.SMAxis^3/data.const.MU_EARTH);

% Initial conditions for state array:
% GPE:
data.ode.Y0GPE = [data.orbit.SMAxis; data.orbit.eccentricity; 
                data.orbit.inclination*pi/180; data.orbit.RAAN*pi/180;
                data.orbit.argPerigee*pi/180; data.orbit.theta0*pi/180]; 
            
% Full state array:
data.ode.Y0 = [zeros(6,1); data.ode.Y0GPE; 0];

% Set the initial position of the flow control valve and the equilibrium
% position of the spring such that the thrust equals the drag at t = 0:
data.FCV.x0 = 0; data.noThrust.switch = 0; data.blockFCV.switch = 0;
data.thruster.noiseSwitch = 0; data.accelerometer.noiseSwitch = 0; 
data.noise = @(t) 0;
[~,out] = odeFun(0,data.ode.Y0,data);

data.FCV.x0 = fzero(@(xFCV) out.dragV + ionThruster(xFCV,data), [0 6e-3]);

data.ode.Y0(2) = data.FCV.x0;

