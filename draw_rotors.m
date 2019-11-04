function draw_rotors(r,R,posicao,frente,pc)
% 
global quad;
if(nargin < 4)
    frente = 0;
    pc = 0;
end
if(nargin < 5)
    pc = 0;
end

x = posicao(1);
y=posicao(2);
z = posicao(3);
 
th = 0:pi/50:2*pi;
 
% xunit = r * cos(th) + x;
% 
% yunit = r * sin(th) + y;
 
xunit = r * cos(th);
 
yunit = r * sin(th);
 
asd = [xunit;yunit;zeros(size(xunit))];
     
for(i=1:length(xunit))
    asd(:,i) = R*asd(:,i) + posicao;
end
 
% h = plot3(xunit, yunit,z*ones(size(xunit)));
if(~frente)
    if(~pc)
        plot3(quad.MainFigure,asd(1,:),asd(2,:),asd(3,:),'k')
    else
        plot3(asd(1,:),asd(2,:),asd(3,:),'k')
    end
else
    if(~pc)
% fill3(quad.MainFigure,asd(1,:),asd(2,:),asd(3,:),'k')
        plot3(quad.MainFigure,asd(1,:),asd(2,:),asd(3,:),'r')
    else
        fill3(asd(1,:),asd(2,:),asd(3,:),'r')
    end
        
    
end

end