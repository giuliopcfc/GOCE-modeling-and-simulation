function [X,Y,Z] = plotOrbit(a,e,i,OM,om,thi,thf,dth,mu)

thi = wrapTo2Pi(thi);
thf = wrapTo2Pi(thf);

if thi > thf
    thf = thf + 2*pi;
end

thvect = [thi:dth:thf];

rmat = zeros(length(thvect),3);


for j = 1:length(thvect)
    [rr,~] = par2car(a,e,i,OM,om,thvect(j),mu);   
    rmat(j,:) = rr; 
end

X = rmat(:,1);
Y = rmat(:,2);
Z = rmat(:,3);
