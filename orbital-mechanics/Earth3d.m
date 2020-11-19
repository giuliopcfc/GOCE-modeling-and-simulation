function Earth3d(Rt)

if nargin==0
    Rt=6378;
end

load('topo.mat','topo','topomap1');
contour(0:359,-89:90,topo,[0 0],'b')
axis equal
hold on
image([0 360],[-90 90],topo,'CDataMapping', 'scaled');
colormap(topomap1)
[x,y,z] = sphere(100);

props.AmbientStrength = 0.1;
props.DiffuseStrength = 1;
props.SpecularColorReflectance = .5;
props.SpecularExponent = 20;
props.SpecularStrength = 1;
props.FaceColor= 'texture';
props.EdgeColor = 'none';
props.FaceLighting = 'phong';
props.Cdata = topo;
surface(x*Rt,y*Rt,z*Rt,props);
light('position',[1 1 1]);
light('position',[-1.5 0.5 -0.5], 'color', [.6 .2 .2]);