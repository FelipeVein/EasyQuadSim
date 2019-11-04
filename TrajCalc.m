function TrajCalc(varargin)
% Trajectory Generation Algorithm
global quad;

if(length(quad.waypoints.T) ==  1)
    errordlg('ERROR - NO WAYPOINTS');
    return 
end
% Final time
quad.Tmax = quad.waypoints.T(end);
%
% Plot
quad.plot.t = 0:quad.dt:quad.Tmax;
%


% [quad.rdes(:,1),quad.rdv(:,1),quad.rda(:,1),quad.rdj(:,1),quad.rds(:,1)]=Planejamento_toda_trajetoria(quad.waypoints.X,quad.dt,quad.waypoints.T);
% [quad.rdes(:,2),quad.rdv(:,2),quad.rda(:,2),quad.rdj(:,2),quad.rds(:,2)]=Planejamento_toda_trajetoria(quad.waypoints.Y,quad.dt,quad.waypoints.T);
% [quad.rdes(:,3),quad.rdv(:,3),quad.rda(:,3),quad.rdj(:,3),quad.rds(:,3)]=Planejamento_toda_trajetoria(quad.waypoints.Z,quad.dt,quad.waypoints.T);
% % %r(:,4) e r(:,5) é para o roll e pitch
% [quad.rdes(:,6),quad.rdv(:,6),quad.rda(:,6),quad.rdj(:,6),quad.rds(:,6)]=Planejamento_toda_trajetoria(quad.waypoints.YAW,quad.dt,quad.waypoints.T);
% 


[quad.rdes(:,1),quad.rdv(:,1),quad.rda(:,1),quad.rdj(:,1),quad.rds(:,1)]=TrajectoryPlanner(quad.waypoints.X,quad.dt,quad.waypoints.T);
[quad.rdes(:,2),quad.rdv(:,2),quad.rda(:,2),quad.rdj(:,2),quad.rds(:,2)]=TrajectoryPlanner(quad.waypoints.Y,quad.dt,quad.waypoints.T);
[quad.rdes(:,3),quad.rdv(:,3),quad.rda(:,3),quad.rdj(:,3),quad.rds(:,3)]=TrajectoryPlanner(quad.waypoints.Z,quad.dt,quad.waypoints.T);
%r(:,4)e r(:,5) é para o roll e pitch
[quad.rdes(:,6),quad.rdv(:,6),quad.rda(:,6),quad.rdj(:,6),quad.rds(:,6)]=TrajectoryPlanner(quad.waypoints.YAW,quad.dt,quad.waypoints.T);
quad.armazena_Desejado = [quad.rdes(:,1) quad.rdes(:,2) quad.rdes(:,3) quad.rdes(:,6)];


quad.rdes = quad.rdes';
quad.rdv = quad.rdv';
quad.rda = quad.rda';
quad.rdj = quad.rdj';
quad.rds = quad.rds';


quad.StartSimulation = 1;

set(quad.StartButton,'Visible','Off');
set(quad.ControllerList,'Visible','Off');
end