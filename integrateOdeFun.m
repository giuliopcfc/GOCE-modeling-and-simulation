function [T,Y,out] = integrateOdeFun(odeFun, tspan, Y0, options, data)

[T,Y] = ode15s(odeFun,tspan,Y0,options,data);

out = struct();
out.thrust = T; out.dragV = T; out.VC = T;

for i = 1:length(T)
    
    [~,outTemp] = odeFun(T(i),Y(i,:)',data);
    
    out.thrust(i) = outTemp.thrust;
    out.dragV(i) = outTemp.dragV;
    out.VC(i) = outTemp.VC;
  
end