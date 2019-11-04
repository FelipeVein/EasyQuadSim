function PlotQuad()


global quad;

try

%%% Rotation and translation of the quadrotor plot
b1 = quad.R*quad.b1 + quad.position;
b2 = quad.R*quad.b2 + quad.position;
b3 = quad.R*quad.b3 + quad.position;
b4 = quad.R*quad.b4 + quad.position;
b5 = quad.R*quad.b5 + quad.position;
%%%
plot3(quad.MainFigure,quad.x_plot,quad.y_plot,quad.z_plot,'k')%black
hold(quad.MainFigure,'on');



%%% Quadrotor 3d Plot
plot3(quad.MainFigure,[b1(1) b3(1)],[b1(2) b3(2)],[b1(3) b3(3)],'k')
plot3(quad.MainFigure,[b2(1) b4(1)],[b2(2) b4(2)],[b2(3) b4(3)],'k')
plot3(quad.MainFigure,[quad.position(1) b5(1)],[quad.position(2) b5(2)],[quad.position(3) b5(3)],'r')

draw_rotors(quad.l/5,quad.R,b1)
draw_rotors(quad.l/5,quad.R,b2)
draw_rotors(quad.l/5,quad.R,b3,1)
draw_rotors(quad.l/5,quad.R,b4)
%%%


plot3(quad.MainFigure,quad.waypoints.X,quad.waypoints.Y,quad.waypoints.Z,'rx')
xlabel(quad.MainFigure,'X');ylabel(quad.MainFigure,'Y');zlabel(quad.MainFigure,'Z');
set(get(quad.MainFigure,'zlabel'),'rotation',0)

% % Inertial-Fixed Figure Area
axis(quad.MainFigure, [-5 5 -5 5 0 5])
% % Quad-Fixed Figure Area
%    axis(quad.MainFigure, [x-1 x+1 y-1 y+1 z-1 z+1])
% % TODO: adaptable figure area
% axis(quad.MainFigure,[quad.xaxis.min quad.xaxis.max quad.yaxis.min quad.yaxis.max quad.zaxis.min quad.zaxis.max])


grid(quad.MainFigure,'on');


%%% Plot Desired Trajectory
if(length(quad.rdes_plot) > 2)
    plot3(quad.MainFigure,quad.rdes_plot(1,:),quad.rdes_plot(2,:),quad.rdes_plot(3,:),'b--');
end
%%%
hold(quad.MainFigure,'on');


plot3(quad.MainFigure,quad.x_des_plot,quad.y_des_plot,quad.z_des_plot,'k--','linewidth',1.4);
%%%

%%% Plot Trajectory
plot3(quad.MainFigure,quad.x_plot,quad.y_plot,quad.z_plot,'k') %black
%%%
hold(quad.MainFigure,'off');


% while moving, plot this
if(quad.iteracao>1)
    plot(quad.Figure_x,0:quad.dt:quad.dt*quad.iteracao,quad.x_plot,'b')
    hold(quad.Figure_x,'on');
    plot(quad.Figure_x,0:quad.dt:quad.dt*(quad.iteracao-1),quad.rdes(1,1:quad.iteracao),'r--')
    grid(quad.Figure_x,'on');
%   axis(quad.Figure_x,[0 quad.dt*(quad.iteracao-1) min(quad.rdes(1,:))-1 max(quad.rdes(1,:))+1])
    axis (quad.Figure_x,[0 quad.waypoints.T(end) min(quad.rdes(1,:))-1 max(quad.rdes(1,:))+1])
    %xlabel(quad.Figure_x,'Tempo');
    ylabel(quad.Figure_x,'X   ');
    hold(quad.Figure_x,'off');
    set(quad.Figure_x,'XTickLabel',[])
    set(get(quad.Figure_x,'ylabel'),'rotation',0)
        
    plot(quad.Figure_y,0:quad.dt:quad.dt*quad.iteracao,quad.y_plot,'b')
    hold(quad.Figure_y,'on');
    plot(quad.Figure_y,0:quad.dt:quad.dt*(quad.iteracao-1),quad.rdes(2,1:quad.iteracao),'r--')
    grid(quad.Figure_y,'on');
    %axis(quad.Figure_y,[0 quad.dt*(quad.iteracao-1) min(quad.rdes(2,:))-1 max(quad.rdes(2,:))+1])
    axis (quad.Figure_y,[0 quad.waypoints.T(end) min(quad.rdes(2,:))-1 max(quad.rdes(2,:))+1])
    %xlabel(quad.Figure_y,'Tempo');
    ylabel(quad.Figure_y,'Y   ');
    hold(quad.Figure_y,'off');
    set(quad.Figure_y,'XTickLabel',[])
    set(get(quad.Figure_y,'ylabel'),'rotation',0)
 %   
    plot(quad.Figure_z,0:quad.dt:quad.dt*quad.iteracao,quad.z_plot,'b')
    hold(quad.Figure_z,'on');
    plot(quad.Figure_z,0:quad.dt:quad.dt*(quad.iteracao-1),quad.rdes(3,1:quad.iteracao),'r--')
    grid(quad.Figure_z,'on');
    %axis(quad.Figure_z,[0 quad.dt*(quad.iteracao-1) min(quad.rdes(3,:))-1 max(quad.rdes(3,:))+1])
    axis (quad.Figure_z,[0 quad.waypoints.T(end) min(min(quad.rdes(3,:))-1,0) max(quad.rdes(3,:))+1])
    %xlabel(quad.Figure_z,'Tempo');
    ylabel(quad.Figure_z,'Z   ');
    hold(quad.Figure_z,'off');
    set(quad.Figure_z,'XTickLabel',[])
    set(get(quad.Figure_z,'ylabel'),'rotation',0)
 %   
    plot(quad.Figure_roll,0:quad.dt:quad.dt*quad.iteracao,quad.roll_plot,'b')
    hold(quad.Figure_roll,'on');
    plot(quad.Figure_roll,0:quad.dt:quad.dt*(quad.iteracao-1),quad.rdes(4,1:quad.iteracao),'r--')
    grid(quad.Figure_roll,'on');
    %axis(quad.Figure_roll,[0 quad.dt*(quad.iteracao-1) min(quad.rdes(4,:))-0.1 max(quad.rdes(4,:))+0.1])
    axis (quad.Figure_roll,[0 quad.waypoints.T(end) -pi pi])
   % xlabel(quad.Figure_roll,'Tempo');
    ylabel(quad.Figure_roll,'Roll    ');
    hold(quad.Figure_roll,'off');
    set(quad.Figure_roll,'XTickLabel',[])
    set(get(quad.Figure_roll,'ylabel'),'rotation',0)
%    
    plot(quad.Figure_pitch,0:quad.dt:quad.dt*quad.iteracao,quad.pitch_plot,'b')
    hold(quad.Figure_pitch,'on');
    plot(quad.Figure_pitch,0:quad.dt:quad.dt*(quad.iteracao-1),quad.rdes(5,1:quad.iteracao),'r--')
    grid(quad.Figure_pitch,'on');
    %axis(quad.Figure_pitch,[0 quad.dt*(quad.iteracao-1) min(quad.rdes(5,:))-0.1 max(quad.rdes(5,:))+0.1])
    axis (quad.Figure_pitch,[0 quad.waypoints.T(end) -pi pi])
    %xlabel(quad.Figure_pitch,'Tempo');
    ylabel(quad.Figure_pitch,'Pitch     ');
    hold(quad.Figure_pitch,'off');
    set(quad.Figure_pitch,'XTickLabel',[])
    set(get(quad.Figure_pitch,'ylabel'),'rotation',0)
    %     
    plot(quad.Figure_yaw,0:quad.dt:quad.dt*quad.iteracao,quad.yaw_plot,'b')
    hold(quad.Figure_yaw,'on');
    plot(quad.Figure_yaw,0:quad.dt:quad.dt*(quad.iteracao-1),quad.rdes(6,1:quad.iteracao),'r--')
    grid(quad.Figure_yaw,'on');
    %axis(quad.Figure_yaw,[0 quad.dt*(quad.iteracao-1) min(quad.rdes(6,:))-0.1 max(quad.rdes(6,:))+0.1])
    axis (quad.Figure_yaw,[0 quad.waypoints.T(end) -7 7])
   % xlabel(quad.Figure_yaw,'Tempo');
    ylabel(quad.Figure_yaw,'Yaw    ');
    hold(quad.Figure_yaw,'off');
    set(quad.Figure_yaw,'XTickLabel',[])
    set(get(quad.Figure_yaw,'ylabel'),'rotation',0)
    %
%    plot(quad.Figure_CSI,quad.CSI,'b')
    plot(quad.Figure_CSI,0:quad.dt:quad.dt*(quad.iteracao-1),quad.CSI,'b')
    hold(quad.Figure_CSI,'on');
    plot(quad.Figure_CSI,0:quad.dt:quad.dt*(quad.iteracao-1),2*ones(length(quad.CSI),1),'r--')
    hold(quad.Figure_CSI,'on');
    grid(quad.Figure_CSI,'on');
    axis (quad.Figure_CSI,[0 quad.waypoints.T(end) 0 2.5]) 
    xlabel(quad.Figure_CSI,'Time');
    ylabel(quad.Figure_CSI,'Psi    ');
    hold(quad.Figure_CSI,'off');
    set(get(quad.Figure_CSI,'ylabel'),'rotation',0)
        
else % While not moving, plot this
    plot(quad.Figure_x,[0],[0])
    axis(quad.Figure_x,[0 1 -1 1])
    ylabel(quad.Figure_x,'X   ');
    grid(quad.Figure_x,'on')
    set(quad.Figure_x,'XTickLabel',[])
    set(get(quad.Figure_x,'ylabel'),'rotation',0)
    plot(quad.Figure_y,[0],[0])
    axis(quad.Figure_y,[0 1 -1 1])
    ylabel(quad.Figure_y,'Y   ');
    grid(quad.Figure_y,'on')
    set(quad.Figure_y,'XTickLabel',[])
    set(get(quad.Figure_y,'ylabel'),'rotation',0)
    plot(quad.Figure_z,[0],[0])
    axis(quad.Figure_z,[0 1 -1 1])
    ylabel(quad.Figure_z,'Z   ');
    grid(quad.Figure_z,'on')
    set(quad.Figure_z,'XTickLabel',[])
    set(get(quad.Figure_z,'ylabel'),'rotation',0)
    plot(quad.Figure_roll,[0],[0])
    axis(quad.Figure_roll,[0 1 -1 1])
    ylabel(quad.Figure_roll,'Roll    ');
    grid(quad.Figure_roll,'on')
    set(quad.Figure_roll,'XTickLabel',[])
    set(get(quad.Figure_roll,'ylabel'),'rotation',0)
    plot(quad.Figure_pitch,[0],[0])
    axis(quad.Figure_pitch,[0 1 -1 1])
    ylabel(quad.Figure_pitch,'Pitch     ');
    grid(quad.Figure_pitch,'on')
    set(quad.Figure_pitch,'XTickLabel',[])
    set(get(quad.Figure_pitch,'ylabel'),'rotation',0)
    plot(quad.Figure_yaw,[0],[0])
    axis(quad.Figure_yaw,[0 1 -1 1])
    ylabel(quad.Figure_yaw,'Yaw    ');
    grid(quad.Figure_yaw,'on')
    set(quad.Figure_yaw,'XTickLabel',[])
    set(get(quad.Figure_yaw,'ylabel'),'rotation',0)
    plot(quad.Figure_CSI,[0],[0])
    axis(quad.Figure_CSI,[0 1 -1 1])
    ylabel(quad.Figure_CSI,'Psi    ');
    grid(quad.Figure_CSI,'on')
    xlabel(quad.Figure_CSI,'Time');
    set(get(quad.Figure_CSI,'ylabel'),'rotation',0)
    drawnow
end
catch
    quad.StartSimulation = 2; % end mainLoop
end
% axis(quad.MainFigure,[quad.xaxis.min quad.xaxis.max quad.yaxis.min quad.yaxis.max quad.zaxis.min quad.zaxis.max])

end