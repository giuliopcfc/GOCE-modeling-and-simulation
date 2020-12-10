function saveFigAsPdf(filename,latexScale,varargin)

WHratio = 4/3;

if nargin == 3
    WHratio = varargin{1};
end

h = gcf();
set(h,'Units','Inches');
pos = get(h,'Position');
pos(3) = pos(4)*WHratio;
set(h,'Position',pos);

width = pos(3);
height = pos(4);
defaultWidth = 5.8333;
defaultHeight =  4.3750;
scale = ((width*height)/(defaultWidth*defaultHeight));

latexLineWidth = 1;
latexFontSize = 9;
c = h.Children;
for i = 1:length(c)
    if strcmp(c(i).Type,'axes')
        set(c(i),'fontsize', latexFontSize/latexScale*scale);
        set(get(c(i),'Children'),'linewidth', latexLineWidth/latexScale*scale);
    end
end


set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,filename,'-dpdf','-r0')
end