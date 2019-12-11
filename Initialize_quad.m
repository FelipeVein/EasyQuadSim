function Initialize_quad()
global quad 





if(~isfield(quad,'ApagarWaypoints'))
    % No pre-chosen trajectory 
    
    quad.waypoints.X = [0];
    quad.waypoints.Y= [0];
    quad.waypoints.Z = [0];
    quad.waypoints.YAW = [0];
    dtaux = 0.2;
    quad.waypoints.T = [0];
    
    quad.orientation = [0;0;0];
%     

    % Trajectory IFAC paper

%     quad.waypoints.X = [0 1 1 2 3 0];
%     quad.waypoints.Y = [0 1 2 3 2 0];
%     quad.waypoints.Z = [0 10 10 10 5 0];
%     quad.waypoints.YAW = [0 pi/2 0 -pi/2 0 0];
%     quad.waypoints.T = [0 4 8 12 16 20];
%     quad.orientation = [0;0;0];
    

    % Trajectory 1 LARS paper 

    % quad.waypoints.X = [0.00,0.00,0.20,0.40,0.60,0.80,1.00,1.20,1.40,1.60,1.80,1.80]';
    % quad.waypoints.Y= [0.00,0.00,0.00,0.40,0.00,-0.40,0.00,0.40,0.00,-0.40,0.00,0.00]'; 
    % quad.waypoints.Z = [0.00,1.00,1.60,1.00,0.40,1.00,1.60,1.00,0.40,1.00,1.60,1.60]';
    % quad.waypoints.YAW = [0.00,0.00,0.00,pi/2,pi,3*pi/2,2*pi,5*pi/2,3*pi,7*pi/2,4*pi,4*pi]'; 
    % dtaux = 0.6;
    % quad.waypoints.T = 0:dtaux:dtaux*12;
    % quad.waypoints.T(2:end-1)=quad.waypoints.T(3:end);
    % quad.waypoints.T=quad.waypoints.T(1:end-1);
    % quad.waypoints.T(end) = quad.waypoints.T(end) - mod(quad.waypoints.T(end),0.02);
    % 
    % quad.orientation = [0;0;0];


    % Trajectory 2 LARS paper
    %do modelo
    % quad.waypoints.X = [-0.40,0.00,0.20,0.40,0.60,0.80,1.00,1.20,1.40,1.60,1.80,1.80]';
    % quad.waypoints.Y= [0.00,0.00,0.00,0.40,0.00,-0.40,0.00,0.40,0.00,-0.40,0.00,0.00]'; 
    % quad.waypoints.Z = [2.00,2.00,2.60,2.00,1.40,2.00,2.60,2.00,1.40,2.00,2.60,2.60]';
    % quad.waypoints.YAW = [0.00,0.00,0.00,pi/2,pi,3*pi/2,2*pi,5*pi/2,3*pi,7*pi/2,4*pi,4*pi]'; 
    % quad.orientation = [deg2rad(88) 0 0]';
    % 
    % dtaux = 0.6;
    % quad.waypoints.T = 0:dtaux:dtaux*12;
    % quad.waypoints.T(2:end-1)=quad.waypoints.T(3:end);
    % quad.waypoints.T=quad.waypoints.T(1:end-1);
    % quad.waypoints.T(end) = quad.waypoints.T(end) - mod(quad.waypoints.T(end),0.02);


    % ´8´ trajectory
    % quad.waypoints.X=[0 0.1172 0.4 0.6828 0.9656 1.2486 1.5314 1.8142 1.9314 1.8142 1.5314 1.2486 0.9656 0.6828 0.4 0.1172 0 0.1172 0.4 0.6828 0.9656 1.2486 1.5314 1.8142 1.9314 1.8142 1.5314 1.2486 0.9656 0.6828 0.4 0.1172 0 0];
    % quad.waypoints.Y=[0 -0.2828 -0.4 -0.2828 0 0.2828 0.4 0.2828 0 -0.2828 -0.4 -0.2828 0 0.2828 0.4 0.2828 0 -0.2828 -0.4 -0.2828 0 0.2828 0.4 0.2828 0 -0.2828 -0.4 -0.2828 0 0.2828 0.4 0.2828 0 0];
    % quad.waypoints.Z=[2 1.5758 1.4 1.5758 2 2.4242 2.6 2.4242 2 1.5758 1.4 1.5758 2 2.4242 2.6 2.4242 2 1.5758 1.4 1.5758 2 2.4242 2.6 2.4242 2 1.5758 1.4 1.5758 2 2.4242 2.6 2.4242 2 2];
    % quad.waypoints.YAW=[-pi/2 -pi/4 0 pi/4 pi/4 pi/4 0 -pi/4 -pi/2 -3*pi/4 -pi -5*pi/4 -5*pi/4 -5*pi/4 -pi -3*pi/4 -pi/2 -pi/4 0 pi/4 pi/4 pi/4 0 -pi/4 -pi/2 -3*pi/4 -pi -5*pi/4 -5*pi/4 -5*pi/4 -pi -3*pi/4 -pi/2 -pi/2];
    % dtaux = 0.4;
    % quad.waypoints.T = 0:dtaux:dtaux*(length(quad.waypoints.X)-1);
    % quad.waypoints.T(end) = quad.waypoints.T(end) - mod(quad.waypoints.T(end),0.02);
    % 
    % quad.orientation = [deg2rad(0) 0 -pi/2]';
    
    
    
    quad.initial_orientation = quad.orientation;
    
end


quad.position = [quad.waypoints.X(1);quad.waypoints.Y(1);quad.waypoints.Z(1)];      %x, y, z (r) inicial
quad.pos_aux = [0;0;0];
% quad.orientation = [0;0;quad.waypoints.YAW(1)]; %roll, pitch, yaw (n)
quad.linear_velocity = [0;0;0]; %vx, vy, vz (rdot)
quad.angular_velocity_inertial_frame = [0;0;0]; %wx, wy, wz (ndot)
% diff(roll,pitch,yaw) com correcao matriz T
quad.angular_velocity_quad_frame= [0;0;0]; %p, q, r (w)
quad.linear_acceleration = [0;0;0]; %ax, ay, az (r2dot)
quad.angular_acceleration = [0;0;0]; %wxdot, wydot, wzdot (n2dot)
quad.angular_acceleration_quad_frame = [0;0;0]; %wdot = diff(p;q;r)
%atribuicao dos valores
roll = quad.orientation(1);
pitch = quad.orientation(2);
yaw = quad.orientation(3);





% Erase waypoints
if(isfield(quad,'ApagarWaypoints'))
    if(quad.ApagarWaypoints)
        quad.waypoints.X = quad.position(1);
        quad.waypoints.Y = quad.position(2);
        quad.waypoints.Z = quad.position(3);
        quad.waypoints.YAW = quad.orientation(3);
        quad.waypoints.T = 0;
    else
        quad.orientation = quad.initial_orientation;
    end

end

%%% Rotation Matrix
quad.R = [cos(yaw)*cos(pitch)-sin(roll)*sin(yaw)*sin(pitch), -cos(roll)*sin(yaw), cos(yaw)*sin(pitch)+cos(pitch)*sin(roll)*sin(yaw);
    cos(pitch)*sin(yaw)+cos(yaw)*sin(roll)*sin(pitch), cos(roll)*cos(yaw), sin(yaw)*sin(pitch)-cos(pitch)*sin(roll)*cos(yaw);
    -cos(roll)*sin(pitch), sin(roll), cos(roll)*cos(pitch)];

%[p;q;r] = T*(euler rates)= wx,wy,wz 
quad.T = [cos(pitch), 0, -cos(roll)*sin(pitch);
        0, 1, sin(roll);
        sin(pitch), 0, cos(roll)*cos(pitch)];    
quad.T10 = zeros(3,3);
    

quad.CSI = []; 


quad.m = 0.468; % quad mass
quad.g = 9.81; % gravity
quad.dt = 0.02; % sampling time 
%%% Intertia
quad.Ixx = 4.856*10^-3;
quad.Iyy = quad.Ixx;
quad.Izz = 8.801*10^-3;
quad.I = [quad.Ixx 0 0;0 quad.Iyy 0; 0 0 quad.Izz];%matrix
%%%
%%% arm length
quad.l = 0.225;
%%%
quad.b = 1.14*10^-7; 
quad.k = 2.98*10^-6; 

%%% air resistance term
quad.Ax = 0.25; 
quad.Ay = 0.25; 
quad.Az = 0.25; 
%%%
%%% rotor min and max
quad.rotor_max_speed = 8500; % rpm
quad.max_force = quad.k*(quad.rotor_max_speed*2*pi/60)^2; % rotor max force
quad.rotor_min_speed = 1300; % rpm
quad.min_force = quad.k*(quad.rotor_min_speed*2*pi/60)^2; % rotor min force
%%%
quad.u1novo_anterior = 0;
quad.u2novo_anterior =[0;0;0];
quad.armazena_Medidos =[];
%12 states: x;y;z; roll;pitch;yaw;  vx;vy;vz;  p;q;r (r n rdot w)
quad.states = [quad.position;quad.orientation;quad.linear_velocity;quad.angular_velocity_quad_frame];
quad.measured_states = quad.states;

quad.rotor_speed = [0;0;0;0]; 
quad.rotor_speed_prev=[0;0;0;0];


% Controller gains
quad.K = [6.1405    9.1331    6.1405    9.1331    6.1405    9.1331    1.5782    1.5782    1.5782    0.8000    0.8000    0.8000];

quad.Krollp = quad.K(1); %Kr(lin) = 6.14 (eq45 do artigo)
quad.Krolld = quad.K(2);  %Kw(lin) = 9.13 (Eq 45 do artigo)
quad.Kpitchp = quad.K(3);%Kr(lin) = 6.14 (eq45 do artigo)
quad.Kpitchd = quad.K(4); %Kw(lin) = 9.13 (Eq 45 do artigo)
quad.Kyawp = quad.K(5);   %Kr(lin) = 6.14 (eq45 do artigo)
quad.Kyawd = quad.K(6);  %Kw(lin) = 9.13 (Eq 45 do artigo)
quad.Kcp = quad.K(7:9)';   %Kcp(lin e naolin) = 1.58 (Eq 45 do artigo)
quad.Kcd = quad.K(10:12)'; %correto Kcd(lin e nlin) =0.8  (Eq 45 do artigo)

%Ganhos(n linear) Kr(para angulos) e Kw(para p q r) EQ 46 artigo
quad.Kr = 9656.655720288503; %proporcional controle de orientation 
quad.Kw = 1832.403541405957; %derivativo controle de orientation


% Para plots
%Quem é b1,b2,b3,b4? os bracos do quadrator em relacao ao frame B
quad.b1 = [-quad.l;0;0];
quad.b2 = [0;-quad.l;0];
quad.b3 = [quad.l;0;0];
quad.b4 = [0;quad.l;0];
quad.b5 = [0;0;quad.l/4];

%apenas atribuicao 
quad.x_plot = [quad.position(1)];
quad.y_plot = [quad.position(2)];
quad.z_plot = [quad.position(3)];
quad.roll_plot = quad.orientation(1);
quad.pitch_plot = quad.orientation(2);
quad.yaw_plot = quad.orientation(3);

%des = desejado? Sim, inicialmente nao se tem trajetoria desejada
%sera preenchido posteriormente por CalcularTrajetoria
quad.x_des_plot = [];
quad.y_des_plot = [];
quad.z_des_plot = [];
quad.roll_des_plot = [];
quad.pitch_des_plot = [];
quad.yaw_des_plot = [];

quad.R_plot= [];
%
%antes de apertar o botao calcular trajetoria
%momento de marcar os Way points
quad.StartSimulation = 0;


% PARA CONTROLADOR

quad.iteracao=1;

%Calculado em CalcularTrajetoria (otima= minimo snap) 
quad.rdes = [];%position
quad.rdv = []; %velocidade desejado
quad.rda = []; %aceleracao desejado
quad.rdj = []; %jerk desejado
quad.rds = []; %snap desejado

quad.pqrc = [0;0;0];  % p q r (controlado)=(desejado)

%vetor linha de zeros. 
%Posteriormente, atribui variaveis desejaveis de CalcularTrajetoria
quad.rc = zeros(6,1);   % x y z roll pitch yaw (controlados)
quad.rc_anterior = zeros(6,1);
quad.pqrc_anterior = zeros(3,1); %p q r (controlados)

quad.T_medido = [cos(pitch), 0, -cos(roll)*sin(pitch);
    0, 1, sin(roll);
    sin(pitch), 0, cos(roll)*cos(pitch)];     %[p;q;r] = T*(euler rates)






% 
% 
% %atan2(sinx,cosx) vai retornar o angulo x
% %Para quad.orientation(1) temos angulo roll base
% quad.orientation(1) = atan2(quad.R(3,2),quad.R(2,2)/cos(quad.orientation(3)));
% quad.orientation(2) = atan2(quad.R(1,3)*cos(quad.orientation(3))+sin(quad.orientation(3))*quad.R(2,3), quad.R(3,3)/cos(quad.orientation(1)));
    


%%% initialize trajectory variables
quad.rdes = [];
quad.rdv  = [];
quad.rda = [];
quad.rdj = [];
quad.rds = [];
quad.rdes_plot = [];
%%%

quad.xaxis.min = -5;
quad.xaxis.max = 5;
quad.yaxis.min = -5;
quad.yaxis.max = 5;
quad.zaxis.min = 0;
quad.zaxis.max = 3;


%configura os limites do grafico 3D da janela principal

% zmin = min(quad.waypoints.Z);
% if zmin-0.5<0 zmin=0; end;
% zmin = 0;
% aux1 = max(quad.waypoints.X) - min(quad.waypoints.X);% -0.4 2
% aux2 = (max(quad.waypoints.Y) - min(quad.waypoints.Y)); % 0.4 -0.4
% aux3 = (max(quad.waypoints.Z) - zmin);
% aux = max([aux1 aux2 aux3]);
% aux4 = mean([min(quad.waypoints.Y)  max(quad.waypoints.Y)]);
% aux5 = (max(quad.waypoints.X)-min(quad.waypoints.X))/2;
% aux6 = mean([min(quad.waypoints.X)  max(quad.waypoints.X)]);
% aux7 = (max(quad.waypoints.Y)-min(quad.waypoints.Y))/2;
% aux8 = mean([zmin  max(quad.waypoints.Z)]);
% aux9 = (max(quad.waypoints.Z)-zmin)/2;
% 
% if aux1 == aux
%     quad.xaxis.min = min(quad.waypoints.X) - 0.5;
%     quad.xaxis.max = max(quad.waypoints.X) + 0.5;
%     quad.yaxis.min = (1.5*aux4 - aux5 - 0.5)/8.7*7;
%     quad.yaxis.max = (1.5*aux4 + aux5 + 0.5)/8.7*7;
%     quad.zaxis.min = (1.5*aux8 - aux5 - 0.5)/8.7*6.1;
%     quad.zaxis.max = (1.5*aux8 + aux5 + 0.5)/8.7*6.1;
% else
%     if aux2 == aux
%         quad.yaxis.min = min(quad.waypoints.Y) - 0.5;
%         quad.yaxis.max = max(quad.waypoints.Y) + 0.5;
%         quad.xaxis.min = (aux6/2 - aux7 -0.5)/7*8.7;
%         quad.xaxis.max = (aux6/2 + aux7 +0.5)/7*8.7;
%         quad.zaxis.min = (aux8/2 - aux7 -0.5)/7*6.1;
%         quad.zaxis.max = (aux8/2 + aux7 +0.5)/7*6.1;
%     else
%         quad.zaxis.min = zmin - 0.5;
%         quad.zaxis.max = max(quad.waypoints.Z) + 0.5;
%         quad.xaxis.min = (1.5*aux6 - aux8 - 0.5)/6.1*8.7;
%         quad.xaxis.max = (1.5*aux6 + aux8 + 0.5)/6.1*8.7;
%         quad.yaxis.min = (1.5*aux4 - aux8 - 0.5)/6.1*7;
%         quad.yaxis.max = (1.5*aux4 + aux8 + 0.5)/6.1*7;
%     end        
% end
% quad.zaxis.min = 0;
end