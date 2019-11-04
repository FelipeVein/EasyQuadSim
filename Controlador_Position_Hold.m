function Controlador_Position_Hold()


global quad;

quad.rdes(:,end+1) = [quad.rdes(1:3,end);0;0;quad.rdes(6,end)];
quad.rdv(:,end+1) = zeros(6,1);
quad.rda(:,end+1) = zeros(6,1);
quad.rdj(:,end+1) = zeros(6,1);
%quad.rds(:,end+1) = zeros(6,1);



end