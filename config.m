data = struct();

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
data.accelerometer.kP = 1e6; % Controller Proportional Gain
data.accelerometer.kD = 5e4; % Controller Derivative Gain

% Flow Control Valve:
data.valve.area = 1e-5; % Orifice Area [m^2]
data.valve.spoolMass = 2e-1; % Spool Mass [kg]
data.valve.kP = 0.1; % Controller Proportional Gain
data.valve.kI = 3; % Controller Integral Gain
data.valve.kSpring = 7e3; % Spring Coefficient [N/m]
data.valve.c = 30; % Friction Coefficient [Ns/m]

% Ion Thruster:
data.thruster.T2 = 240; % Xenon Working Temperature [K]
data.thruster.p2 = 2e5; % Xenon Working Pressure [Pa]
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