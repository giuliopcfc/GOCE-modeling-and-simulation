function thrust = ionThruster(xFCV,data)

areaFCV_A0 = data.FCV.A0;
diameterFCV = sqrt(4*areaFCV_A0/pi);

% assigning the value to z
z = (diameterFCV-xFCV)/diameterFCV;

% computation of the orifice area
alpha = 2*acos(1-2*z);
areaOrifice = data.D0^2/8*(alpha - sin(alpha));

% since the pressure after the valve is much lower than the critical
% pressure, the flow is chocked. Hypothesis of homoentropic flow.
T2 = data.thruster.T2;
p2 = data.thruster.p2;
k =  data.thruster.kXe; 
R = data.const.Ru/data.thruster.mmXe;

mDot = areaOrifice*p2*sqrt(k)/sqrt(T2*R)*(1+0.5*(k-1))^((-k-1)/2*(k-1));

vAcc = sqrt(2*data.thruster.e*data.thruster.deltaV/data.thruster.massIon);

thrust = mDot*vAcc;


end