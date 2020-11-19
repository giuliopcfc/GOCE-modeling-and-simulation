function rho = density(rr,data)

R_EQ = data.const.R_EQUATORIAL;
R_PO = data.const.R_POLAR;
% latitude of the satellite
theta = pi/2 - acos(rr(3)/norm(rr));

a = R_PO*cos(theta);
b = R_EQ*sin(theta);

% Earth's radius for the specified latitude
R = sqrt((R_EQ^2*R_PO^2)/(a^2+b^2));

alt = norm(rr) - R;

if alt<=180
    h0 = 150;
    rho0 = 2.070*1e-9;
    H = 22.523;
elseif alt<=200 && alt>180
    h0 = 180;
    rho0 = 5.464*1e-10;
    H = 29.740;
elseif alt<=250 && alt>200
    h0 = 200;
    rho0 = 2.789*1e-10;
    H = 37.105;
elseif alt<=300 && alt>250
    h0 = 250;
    rho0 = 7.248*1e-11;
    H = 45.546;  
elseif alt<=350 && alt>300
    h0 = 300;
    rho0 = 2.418*1e-11;
    H = 53.628;
elseif alt<=400 && alt>350
    h0 = 350;
    rho0 = 9.158*1e-12;
    H = 53.298;
elseif alt<=450 && alt>400
    h0 = 400;
    rho0 = 3.725*1e-12;
    H = 58.515;
elseif alt<=500 && alt>450
    h0 = 450;
    rho0 = 1.585*1e-12;
    H = 60.828;
elseif alt<=600 && alt>500
    h0 = 500;
    rho0 = 6.967*1e-13;
    H = 63.822;
elseif alt>600
    h0 = 600;
    rho0 = 1.454*1e-13;
    H = 71.835;
else 
    rho0 = 0;
end

rho = rho0*exp(-(alt-h0)/H);

end
