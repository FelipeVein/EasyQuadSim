function SaveSimulation(varargin)

%SUAVE(100%)
global quad;
salvar = quad; 
%slava celula com a string
arquivo = inputdlg('Salvar em qual arquivo?','Salvar',1,{'.mat'});
if(~isempty(arquivo))
    arquivo = arquivo{1}; %% porque a funcao inputdlg dá resultado do tipo cell, e não string. Essa linha transforma de cell para string
    
    %era vetores diferentes de tamanho, ai apagou o primeiro termo to work
    salvar.x_plot(1) = [];
    salvar.y_plot(1) = [];
    salvar.z_plot(1) = [];
    salvar.roll_plot(1) = [];
    salvar.pitch_plot(1) = [];
    salvar.yaw_plot(1) = [];
    
    %criar vetor de tempo
    salvar.tempo_plot = 0:salvar.dt:salvar.dt*(salvar.iteracao-1);
    
    %nao salvar as janelas em si (remove field)
    salvar = rmfield(salvar,{'Figure_x','Figure_y','Figure_z','Figure_roll','Figure_pitch','Figure_yaw','rc_anterior','pqrc_anterior','pqrc','rc','rdes','rdv','rda','rdj','rds','BotaoCalcTraj','handler','ControllerList','MainWindow','MainFigure','Tmax','StartSimulation','iteracao','b1','b2','b3','b4','b5','motor','estados','estados_medidos','Ixx','Iyy','Izz','R','T','position','orientation','linear_velocity','angular_velocity_inertial_frame','angular_velocity_quad_frame','linear_acceleration','angular_acceleration','plot','T_medido'});
    %se existir o campo(variavel da struct) ApagarWaypoints
    if(isfield(salvar,'ApagarWaypoints'))
        %nao salvar esse campo da variavel QUAD
        salvar = rmfield(salvar,'ApagarWaypoints');
    end
    save(arquivo,'salvar');
end
end