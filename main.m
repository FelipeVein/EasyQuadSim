function main

clear all
%close all

global quad 
% ´quad´ is a struct that contains every important variable
% let's initialize it
Initialize_quad();

quad.MainWindow = figure('units','normalized','position',[.05 .08 .90 .82],'name','Quadrotor Control Development Platform - Federal Univertsity of Juiz de Fora/Brazil','color','w','numbertitle','off');


quad.MainFigure = axes('units','normalized','position',[.17 .08 .65 .8]);


quad.Figure_x = axes('units','normalized','position',[.87 .89 .115 .1]);
quad.Figure_y = axes('units','normalized','position',[.87 .75 .115 .1]);
quad.Figure_z = axes('units','normalized','position',[.87 .61 .115 .1]);
quad.Figure_roll = axes('units','normalized','position',[.87 .47 .115 .1]);
quad.Figure_pitch = axes('units','normalized','position',[.87 .33 .115 .1]);
quad.Figure_yaw = axes('units','normalized','position',[.87 .19 .115 .1]);
quad.Figure_CSI = axes('units','normalized','position',[.87 .05 .115 .1]);


axis(quad.MainFigure,[-5 5 -5 5 0 3])
axis(quad.Figure_x,[0 5 -5 5])
axis(quad.Figure_y,[0 5 -5 5])
axis(quad.Figure_z,[0 5 -5 5])
axis(quad.Figure_roll,[0 5 -90 90 ])
axis(quad.Figure_pitch,[0 5 -90 90 ])
axis(quad.Figure_yaw,[ 0 5 -180 180])


xlabel(quad.MainFigure,'X');
ylabel(quad.MainFigure,'Y');
zlabel(quad.MainFigure,'Z   ');
set(get(quad.MainFigure,'zlabel'),'rotation',0)

xlabel(quad.Figure_x,'Time');
ylabel(quad.Figure_x,'X');

xlabel(quad.Figure_y,'Time');
ylabel(quad.Figure_y,'Y');

xlabel(quad.Figure_z,'Time');
ylabel(quad.Figure_z,'Z');

xlabel(quad.Figure_roll,'Time');
ylabel(quad.Figure_roll,'Roll');

xlabel(quad.Figure_pitch,'Time');
ylabel(quad.Figure_pitch,'Pitch');

xlabel(quad.Figure_yaw,'Time');
ylabel(quad.Figure_yaw,'Yaw');

quad.handler.X = uicontrol('units','normalized','position',[.06 .85 .06 .04],'style','edit','fontsize',10,'string',2,'backgroundcolor','w');
quad.handler.Y = uicontrol('units','normalized','position',[.06 .75 .06 .04],'style','edit','fontsize',10,'string',0,'backgroundcolor','w');
quad.handler.Z = uicontrol('units','normalized','position',[.06 .65 .06 .04],'style','edit','fontsize',10,'string',0,'backgroundcolor','w');
quad.handler.YAW = uicontrol('units','normalized','position',[.06 .55 .06 .04],'style','edit','fontsize',10,'string',0,'backgroundcolor','w');
quad.handler.T = uicontrol('units','normalized','position',[.06 .45 .06 .04],'style','edit','fontsize',10,'string',0,'backgroundcolor','w');

uicontrol('units','normalized','position',[.02 .85 .03 .04],'style','text','fontsize',10,'string','X','backgroundcolor','w');
uicontrol('units','normalized','position',[.02 .75 .03 .04],'style','text','fontsize',10,'string','Y','backgroundcolor','w');
uicontrol('units','normalized','position',[.02 .65 .03 .04],'style','text','fontsize',10,'string','Z','backgroundcolor','w');
uicontrol('units','normalized','position',[.02 .55 .03 .04],'style','text','fontsize',10,'string','Yaw','backgroundcolor','w');
uicontrol('units','normalized','position',[.01 .45 .04 .03],'style','text','fontsize',10,'string','Time','backgroundcolor','w');


uicontrol('units','normalized','position',[.025 .95 .1 .03],'style','pushbutton','fontsize',10,'string','Save Simulation','callback',@SaveSimulation);
% [.025 .9 .1 .03]
uicontrol('units','normalized','position',[.025 .40 .1 .03],'style','pushbutton','fontsize',10,'string','Set Waypoint','callback',@MarkWaypoint);
% uicontrol('units','normalized','position',[.05 .40 .07 .02],'style','pushbutton','fontsize',10,'string','Marcar Waypoint','callback',@MarcarWaypoint);


% quad.BotaoPSO = uicontrol('units','normalized','position',[.025 .36 .1 .03],'style','pushbutton','fontsize',10,'string','PSO Tunning','callback',@PSOTunning);

uicontrol('units','normalized','position',[.025 .31 .1 .03],'style','pushbutton','fontsize',10,'string','Set Parameters','callback',@ParamConfig);

quad.StartButton = uicontrol('units','normalized','position',[.025 .21 .1 .03],'style','pushbutton','fontsize',10,'string','Start Simulation','callback',@TrajCalc);

uicontrol('units','normalized','position',[.025 .16 .1 .03],'style','pushbutton','fontsize',10,'string','End/Clean Simulation','callback',@ClearSimulation);

addpath('./Controllers');
ControllerFiles = dir('Controllers');
dropdownlist = {ControllerFiles(~[ControllerFiles.isdir]).name};
for i = 1:length(dropdownlist)
dropdownlist{i} = dropdownlist{i}(1:end-2);
end
for i = 1:length(dropdownlist)
    if(isequal(dropdownlist{i}, 'How_to_write_a_controller'))
        dropdownlist(i) = [];
        break
    end
end

quad.ControllerList = uicontrol('units','normalized','position',[0.025 .05 .1 .03],'style','popupmenu','fontsize',10,'string',dropdownlist,'value',1);

PlotQuad();

drawnow
while(~quad.StartSimulation)
    mainLoop();
end
    quad.StartSimulation

end





% quad.StartSimulation: 
% case 0: waiting waypoints
% case 1: running simulation
% case 2: exit program
% case 3: transition 1 -> 0

function mainLoop()
global quad;

Initialize_quad();
while(~quad.StartSimulation)
    drawnow 
    PlotQuad();
    Draw_Trajectory();
end
% Start: StartSimulation = 1

try
    Controller = str2func(quad.ControllerList.String{quad.ControllerList.Value});
catch
    return
end

while(quad.StartSimulation == 1)
   tic %starts measuring time
   
   Measures();
   
   Controller();
%    try
%        if(get(quad.ControllerList,'Value')==1)
%            Controlador();
%        elseif(get(quad.ControllerList,'Value')==2)
%            ControladorNaoLinear();
%        elseif(get(quad.ControllerList,'Value')==3)
%            ControladorNaoLinearArtigo();
%        end
%    catch % GUI Error
%        disp('ERROR at main.m')
%        return
%    end
   Model();
   %
%    try
%     if(quad.iteracao < 5 && quad.iteracao > 1)
%         disp('a')
%         disp(quad.x_plot)
%         disp(quad.y_plot)
%         disp(quad.z_plot)
%     end
        %plota o grafico APENAS de 10 em 10 iteracoes para evitar lentidao
       if(~mod(quad.iteracao,10))
            PlotQuad();
       end
%    catch
%        return
%    end
   drawnow
   %toc é o tempo passado a partir do TIC acima
   %delay toc-  dt(tempo de amostragem)
   %segurar o programa pra nao andar mais rapido que o tempo de amostragem
   while(toc<quad.dt)
   end
   
  quad.iteracao=quad.iteracao+1;
end
% After ending simulation, plot results
if(quad.StartSimulation == 3 && ~isempty(quad.rds))
     limits=[8500*ones(length(quad.rds(1,:)),1) 1300*ones(length(quad.rds(1,:)),1)];
     figure(2)
     subplot(1,4,1)
     hold off
     plot(0.02:0.02:quad.dt*(length(quad.rds(1,:)-1)),quad.rotor_speed_plot(1:length(quad.rds(1,:)),1)*60/(2*pi),'b'); %rotacoes em rpm do motor 1
     hold on
     plot(0.02:0.02:quad.dt*(length(quad.rds(1,:)-1)),limits(:,1),'r--');
     plot(0.02:0.02:quad.dt*(length(quad.rds(1,:)-1)),limits(:,2),'r--');
     max1 = max(abs(quad.rotor_speed_plot(1:length(quad.rds(1,:)),1))*60/(2*pi));
     axis([0 quad.dt*length(quad.rds(1,:)) 0 9000]); grid
     title('N_1')
     xlabel('Time [s]')
     ylabel('Speed [rpm]')
     subplot(1,4,2)
     hold off
     plot(0.02:0.02:quad.dt*(length(quad.rds(1,:)-1)),quad.rotor_speed_plot(1:length(quad.rds(1,:)),2)*60/(2*pi),'b'); %rotacoes em rpm do motor 2figure;
     hold on
     plot(0.02:0.02:quad.dt*(length(quad.rds(1,:)-1)),limits(:,1),'r--');
     plot(0.02:0.02:quad.dt*(length(quad.rds(1,:)-1)),limits(:,2),'r--');
     max2 = max(abs(quad.rotor_speed_plot(1:length(quad.rds(1,:)),2))*60/(2*pi));
     axis([0 quad.dt*length(quad.rds(1,:)) 0 9000]); grid
     title('N_2')
     xlabel('Time [s]')
     ylabel('Speed [rpm]')
     subplot(1,4,3);
     hold off
     plot(0.02:0.02:quad.dt*(length(quad.rds(1,:)-1)),quad.rotor_speed_plot(1:length(quad.rds(1,:)),3)*60/(2*pi),'b'); %rotacoes em rpm do motor 3figure;
     hold on
     plot(0.02:0.02:quad.dt*(length(quad.rds(1,:)-1)),limits(:,1),'r--');
     plot(0.02:0.02:quad.dt*(length(quad.rds(1,:)-1)),limits(:,2),'r--');
     max3 = max(abs(quad.rotor_speed_plot(1:length(quad.rds(1,:)),3))*60/(2*pi));
     axis([0 quad.dt*length(quad.rds(1,:)) 0 9000]); grid
     title('N_3')
     xlabel('Time [s]')
     ylabel('Speed [rpm]')
     subplot(1,4,4);
     hold off
     plot(0.02:0.02:quad.dt*(length(quad.rds(1,:)-1)),quad.rotor_speed_plot(1:length(quad.rds(1,:)),4)*60/(2*pi),'b'); %rotacoes em rpm do motor 4figure;
     hold on
     plot(0.02:0.02:quad.dt*(length(quad.rds(1,:)-1)),limits(:,1),'r--');
     plot(0.02:0.02:quad.dt*(length(quad.rds(1,:)-1)),limits(:,2),'r--');
     max4 = max(abs(quad.rotor_speed_plot(1:length(quad.rds(1,:)),4))*60/(2*pi));
     axis([0 quad.dt*length(quad.rds(1,:)) 0 9000]); grid
     title('N_4')
     xlabel('Time [s]')
     ylabel('Speed [rpm]')
     set (gcf,'Color','white')
     set (gcf,'Position',[10 200 1500 200])
     
%      figure(6);
%      plot(quad.Forcas(:,1));
%      figure(7);
%      plot(quad.Forcas(:,2));
%      figure(8);
%      plot(quad.Forcas(:,3));
%      figure(9);
%      plot(quad.Forcas(:,4));
%      figure(10)
%      plot(quad.Forcasnovas(:,1));
%      figure(11);
%      plot(quad.Forcasnovas(:,2));
%      figure(12);
%      plot(quad.Forcasnovas(:,3));
%      figure(13);
%      plot(quad.Forcasnovas(:,4));
%     figure (14);
%     plot(quad.salva_1);
%     figure (15);
%     plot(quad.salva_2);
%     RMS_da_trajetoria(); 
%     plot(quad.Figure_CSI,quad.CSI,'b')
%     axis (quad.Figure_CSI,[0 length(quad.CSI) 0 2.5])
%     xlabel(quad.Figure_CSI,'Tempo');
%     ylabel(quad.Figure_CSI,'CSI');
%     hold(quad.Figure_CSI,'on');
%     
%     figure(16)
%     plot(quad.CSI);
%     axis([0 length(quad.CSI) 0 2])
 
    quad.StartSimulation = 0;
elseif(quad.StartSimulation == 3 && isempty(quad.rds))
    quad.StartSimulation = 0;
end
end

