function rho = density(rr,data)

alt = norm(rr) - data.const.R_MEAN;

if alt<=300
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
elseif alt<=700 && alt>600
    h0 = 600;
    rho0 = 1.454*1e-13;
    H = 71.835;
elseif alt<=800 && alt>700
    h0 = 700;
    rho0 = 3.614*1e-14;
    H = 88.667;
elseif alt<=900 && alt>800
    h0 = 800;
    rho0 = 1.170*1e-14;
    H = 124.64;
elseif alt<=1000 && alt>900
    h0 = 900;
    rho0 = 5.245*1e-15;
    H = 181.05;
elseif alt>1000
    h0 = 1000;
    rho0 = 3.019*1e-15;
    H = 268;
else 
    rho0 = 0;
end

rho = rho0*exp(-(alt-h0)/H);

end
