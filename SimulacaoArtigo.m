function [erro,rplot,t1,rdes1] = SimulacaoArtigo(K,dtaux,rdes,rdv,rda,roll,pitch,yaw,R)

m = 0.468;
g = 9.81;
%momentos de inercia
Ixx = 4.856*10^-3;
Iyy = Ixx;
Izz = 8.801*10^-3;
%tamanho do braço
l = 0.225;
%parametros
b = 1.140*10^-7;
k = 2.980*10^-6;
Ir = 3.357*10^-5;
% %resistencia do ar
Ax = 0.25;
Ay = 0.25;
Az = 0.25;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%posicao inicial
x = -0.4;
y = 0;
z = 2;
%posicao dos bracos no marco de referência do corpo do quadrotor
braco1 = [-l;0;0];
braco2 = [0;-l;0];
braco3 = [l;0;0];
braco4 = [0;l;0];
braco5 = [0;0;l];
b1 = braco1;
b2 = braco2;
b3 = braco3;
b4 = braco4;
%velocidade linear inicial
vx = 0;
vy = 0;
vz = 0;
%aceleraçao linear inicial
ax = 0;
ay = 0;
az = 0;
espdd=[0;0;0];
% %euler angles inicial
% roll = 0;
% pitch = 0;
% yaw = 0;
%euler rates inicial
rollr = 0;
pitchr = 0;
yawr = 0;
eulerd = [0;0;0];
%angular velocities inicial
p = 0;
q = 0;
r = 0;
%angular accelerations inicial
wax = 0;
way = 0;
waz = 0;
wd = [0;0;0];
%vetor de estados inicial
estados = [x;y;z;roll;pitch;yaw;vx;vy;vz;p;q;r];
%%%%%%%%%%%
%velocidade angular dos motores inicial
motor1 = 0;
motor2 = 0;
motor3 = 0;
motor4 = 0;
motor10 = [0;0;0;0];
%rotação máxima e mínima dos motores
rotacaoMax = 8500;
rotacaoMin = 1300;
%%%
%matriz de inercia inicial
I = [Ixx 0 0;0 Iyy 0; 0 0 Izz];

%matriz de rotacao inicial
% R = [cos(yaw)*cos(pitch)-sin(roll)*sin(yaw)*sin(pitch) -cos(roll)*sin(yaw) cos(yaw)*sin(pitch)+cos(pitch)*sin(roll)*sin(yaw);
%     cos(pitch)*sin(yaw)+cos(yaw)*sin(roll)*sin(pitch) cos(roll)*cos(yaw) sin(yaw)*sin(pitch)-cos(pitch)*sin(roll)*cos(yaw);
%     -cos(roll)*sin(pitch) sin(roll) cos(roll)*cos(pitch)];

x_plot = [x];
y_plot = [y];
z_plot = [z];
roll_plot = roll;
pitch_plot = pitch;
yaw_plot = yaw;

x_des_plot = [x];
y_des_plot = [y];
z_des_plot = [z];

rdes1=rdes;

%erro quadratico médio inicial

erro_qd_medio = [0;0;0;0;0;0];
erro = [0;0;0;0;0;0];
aux = [0;0;0;0;0;0];

%medições simula ruido de medição

estados_medidos = getMeasures(estados);
    
x_plot_medido = [estados_medidos(1)];
y_plot_medido = [estados_medidos(2)];
z_plot_medido = [estados_medidos(3)];
roll_plot_medido = [estados_medidos(4)];
pitch_plot_medido = [estados_medidos(5)];
yaw_plot_medido = [estados_medidos(6)];

%%%%%%%%%%%%
% parâmetros de tempo
dt = 1/50;
Tmax = dtaux*11;
Tmax = Tmax - mod(Tmax,dt);
t1 = 0:dt:Tmax;

%referencias iniciais
rc = zeros(6,1);
rc_anterior = zeros(6,1);
pqrc_anterior = zeros(3,1);

erro_nao_linear = [0;0;0];
erro_nao_linear_anterior = [0;0;0];

%
CSI = [0];

Kcp = K(1:3);
Kcd = K(4:6);
Kr = K(7);
Kw = K(8);

%%--------------------------------INICIA A MALHA--------------------------------%%
a = 0;

u10 = 0;
u20 = 0;
parar = 0;

while a*dt < Tmax && parar==0
    a=a+1;
    
    %%--------------------------------SENSORES simula erro de medição--------------------------------%%
    estados_medidos = getMeasures(estados);
    
%     estados_medidos = estados; %DESCOMENTAR CASO QUEIRA RETIRAR ERROS DE MEDIÇÃO
    
    %%--------------------------------CONTROLADOR--------------------------------%%
    
% %%%%CONTROLADOR LINEAR%%%%%
%     rc = [rda(1:3,a); rdes(4:6,a)];
%     
% %     equacao 15,16,17 do artigo
% %     Kcd e o vetor de ganhos derivativos do erro de velocidade
% %     Kcp é o vetor de ganhos proporcionais do erro de posicao
%     rc(1:3) = rc(1:3) + Kcd'.*(rdv(1:3,a) - estados_medidos(7:9)) + Kcp'.* (rdes(1:3,a) - estados_medidos(1:3));
%   
% %     Equacao 19 20 e 21 do artigo
%     rc(4) = 1/g*(rc(1) * sin(rc(6)) - rc(2) * cos(rc(6)));
%     rc(5) = 1/g*(rc(1) * cos(rc(6)) + rc(2) * sin(rc(6)));
%     
%     if rc(4) > pi
%         rc(4) = mod(rc(4),-pi);
%     end
%     if rc(4) < -pi
%         rc(4) = mod(rc(4),-pi);
%     end
%     
%     if rc(5) > pi
%         rc(5) = mod(rc(5),-pi);
%     end
%     if rc(5) < -pi
%         rc(5) = mod(rc(5),-pi);
%     end
%     
%     R_des = [cos(rc(6))*cos(rc(5))-sin(rc(4))*sin(rc(6))*sin(rc(5)), -cos(rc(4))*sin(rc(6)), cos(rc(6))*sin(rc(5))+cos(rc(5))*sin(rc(4))*sin(rc(6));...
%     cos(rc(5))*sin(rc(6))+cos(rc(6))*sin(rc(4))*sin(rc(5)), cos(rc(4))*cos(rc(6)), sin(rc(6))*sin(rc(5))-cos(rc(5))*sin(rc(4))*cos(rc(6));...
%     -cos(rc(4))*sin(rc(5)), sin(rc(4)), cos(rc(4))*cos(rc(5))];
% 
%     R_medido = [cos(estados_medidos(6))*cos(estados_medidos(5))-sin(estados_medidos(4))*sin(estados_medidos(6))*sin(estados_medidos(5)), -cos(estados_medidos(4))*sin(estados_medidos(6)), cos(estados_medidos(6))*sin(estados_medidos(5))+cos(estados_medidos(5))*sin(estados_medidos(4))*sin(estados_medidos(6));...
%     cos(estados_medidos(5))*sin(estados_medidos(6))+cos(estados_medidos(6))*sin(estados_medidos(4))*sin(estados_medidos(5)), cos(estados_medidos(4))*cos(estados_medidos(6)), sin(estados_medidos(6))*sin(estados_medidos(5))-cos(estados_medidos(5))*sin(estados_medidos(4))*cos(estados_medidos(6));...
%     -cos(estados_medidos(4))*sin(estados_medidos(5)), sin(estados_medidos(4)), cos(estados_medidos(4))*cos(estados_medidos(5))];
%     
%     %%%%%%%%%apenas para criar o plot no final
% 
% % repassando angulos de controle (roll, pitch e yaw)
%     rdes1(4:5,a) = rc(4:5);
% 
%     T_medido = [1 , 0, -estados_medidos(5); 
%                 0, 1, estados_medidos(4);
%                 estados_medidos(5), 0, 1];  
%    
% % equacao 22 onde (rc(4) -rc(4)anterior)/dt = diff ( rc(4) )
% % anterior atualizado na linha 78 
%     pqrc = T_medido* [(rc(4)-rc_anterior(4))/dt;(rc(5)-rc_anterior(5))/dt;(rc(6)-rc_anterior(6))/dt];
% 
% %     Equacao 18 artigo CORRETO
%     u1 = m*(g+rc(3));
%     
%     u2 = [Kr * (rc(4) - estados_medidos(4)) + Kw * (pqrc(1) - estados_medidos(10));
%         Kr * (rc(5) - estados_medidos(5)) + Kw * (pqrc(2) - estados_medidos(11));
%         Kr * (rc(6) - estados_medidos(6)) + Kw * (pqrc(3) - estados_medidos(12))];
% %%%%FIM DO CONTROLADOR LINEAR%%%%%    
  

    
    
%%%CONTROLADORES NÃO LINEARES%%%%%

    rc(1:3) = rda(1:3,a) + Kcd'.*(rdv(1:3,a) - estados_medidos(7:9)) + Kcp'.*(rdes(1:3,a) - estados_medidos(1:3));
    rc(6) = rdes(6,a);
    
    vetor_t = m*rc(1:3) + m*g*[0;0;1];
    
    R_medido = [cos(estados_medidos(6))*cos(estados_medidos(5))-sin(estados_medidos(4))*sin(estados_medidos(6))*sin(estados_medidos(5)), -cos(estados_medidos(4))*sin(estados_medidos(6)), cos(estados_medidos(6))*sin(estados_medidos(5))+cos(estados_medidos(5))*sin(estados_medidos(4))*sin(estados_medidos(6));...
    cos(estados_medidos(5))*sin(estados_medidos(6))+cos(estados_medidos(6))*sin(estados_medidos(4))*sin(estados_medidos(5)), cos(estados_medidos(4))*cos(estados_medidos(6)), sin(estados_medidos(6))*sin(estados_medidos(5))-cos(estados_medidos(5))*sin(estados_medidos(4))*cos(estados_medidos(6));...
    -cos(estados_medidos(4))*sin(estados_medidos(5)), sin(estados_medidos(4)), cos(estados_medidos(4))*cos(estados_medidos(5))];
    
    vetor_t_normalizado = vetor_t/norm(vetor_t);
    
%%%%CONTROLE NÃO LINEAR GEOMETRICO%%%%%

    b1d = [cos(rc(6));sin(rc(6));0];
    b2d = cross(vetor_t_normalizado, b1d)/norm(cross(vetor_t_normalizado, b1d));
    
    R_des = [cross(b2d,vetor_t_normalizado), b2d, vetor_t_normalizado];
    
    rc(4) = atan2(R_des(3,2),R_des(2,2)/cos(rc(6)));
    rc(5) = atan2(-R_des(3,1)*cos(rc(4)), R_des(3,3)/cos(rc(4)));
    
    if(rc(4) > pi) rc(4) = mod(rc(4),-pi);end
    if(rc(4) < -pi)rc(4) = mod(rc(4),-pi);end
    
    if(rc(5) > pi) rc(5) = mod(rc(5),-pi);end
    if(rc(5) < -pi)rc(5) = mod(rc(5),-pi);end
    
    rdes1(4:5,a) = rc(4:5);
    
%erro proporcional à diferença angular
    erro_nao_linear_skew_symmetric = 1/2 * (R_des'*R_medido - R_medido'*R_des);
    erro_nao_linear = [0;0;0];
    erro_nao_linear(1) = erro_nao_linear_skew_symmetric(3,2);
    erro_nao_linear(2) = erro_nao_linear_skew_symmetric(1,3);
    erro_nao_linear(3) = erro_nao_linear_skew_symmetric(2,1);
    
%matriz de transformação das derivadas de roll, pitch e yaw em pqr    
    TRANSFORMACAO = [cos(estados_medidos(5)), 0, -cos(estados_medidos(4))*sin(estados_medidos(5)); ...
        0, 1, sin(estados_medidos(4));...
        sin(estados_medidos(5)), 0, cos(estados_medidos(4))*cos(estados_medidos(5))];  
    
    pqrc = TRANSFORMACAO* [(rc(4)-rc_anterior(4))/dt;(rc(5)-rc_anterior(5))/dt;(rc(6)-rc_anterior(6))/dt];
    
%erro proporcional ao erro de velocidade angular
    erro_nao_linear_w = estados_medidos(10:12) - R_medido' * R_des * pqrc;
    
%derivada das velocidades angulares desejadas
    erro_ac_angular = (pqrc - pqrc_anterior)/dt;
    
    u1 = vetor_t' * (R_medido*[0;0;1]); 
    
%     u2 = cross(estados_medidos(10:12),I*estados_medidos(10:12)) + (-Kr*erro_nao_linear - Kw * erro_nao_linear_w);
    u2 = cross(estados_medidos(10:12),I*estados_medidos(10:12)) + I*(-Kr*erro_nao_linear - Kw * erro_nao_linear_w) - I * (cross(estados_medidos(10:12),R_medido'*R_des*pqrc) - R_medido'*R_des*erro_ac_angular);

%%%%FIM CONTROLE NÃO LINEAR GEOMETRICO%%%%%



% %%%%%CONTROLE NÃO LINEAR%%%%%
% % 
%     rc(4) = atan((vetor_t_normalizado(1) * sin(rc(6)) - vetor_t_normalizado(2) * cos(rc(6)))/vetor_t_normalizado(3));
%     
%     rc(5) = atan2(vetor_t_normalizado(1) * cos(rc(6)) + vetor_t_normalizado(2) * sin(rc(6)), vetor_t_normalizado(3)/cos(rc(4)));
% 
%     if(rc(4) > pi) rc(4) = mod(rc(4),-pi);end
%     if(rc(4) < -pi)rc(4) = mod(rc(4),-pi);end
%     
%     if(rc(5) > pi) rc(5) = mod(rc(5),-pi);end
%     if(rc(5) < -pi)rc(5) = mod(rc(5),-pi);end
%     
%     R_des = [cos(rc(6))*cos(rc(5))-sin(rc(4))*sin(rc(6))*sin(rc(5)), -cos(rc(4))*sin(rc(6)), cos(rc(6))*sin(rc(5))+cos(rc(5))*sin(rc(4))*sin(rc(6));...
%     cos(rc(5))*sin(rc(6))+cos(rc(6))*sin(rc(4))*sin(rc(5)), cos(rc(4))*cos(rc(6)), sin(rc(6))*sin(rc(5))-cos(rc(5))*sin(rc(4))*cos(rc(6));...
%     -cos(rc(4))*sin(rc(5)), sin(rc(4)), cos(rc(4))*cos(rc(5))];
%     
%     vetor_t_teste = R_des * [0;0;1];
%     
%     if abs( vetor_t_teste(3) - vetor_t_normalizado(3) ) > 0.01
%         if rc(4) < -pi
%             rc(4) = mod(rc(4),-pi);
%         end
%         rc(4) = rc(4) - pi;
%         rc(5) = atan2(vetor_t_normalizado(1) * cos(rc(6)) + vetor_t_normalizado(2) * sin(rc(6)), vetor_t_normalizado(3)/cos(rc(4)));
%     end
%     
%     rdes1(4:5,a) = rc(4:5);
%     
%     delta_R = R_des'*R_medido;
% 
%     theta = acos(( trace(delta_R) - 1)/2);
%     vetorK(1,1) = (delta_R(3,2) - delta_R(2,3))/sqrt((delta_R(3,2) - delta_R(2,3))^2+(delta_R(1,3) - delta_R(3,1))^2+(delta_R(2,1) - delta_R(1,2))^2);
%     vetorK(2,1) = (delta_R(1,3) - delta_R(3,1))/sqrt((delta_R(3,2) - delta_R(2,3))^2+(delta_R(1,3) - delta_R(3,1))^2+(delta_R(2,1) - delta_R(1,2))^2);
%     vetorK(3,1) = (delta_R(2,1) - delta_R(1,2))/sqrt((delta_R(3,2) - delta_R(2,3))^2+(delta_R(1,3) - delta_R(3,1))^2+(delta_R(2,1) - delta_R(1,2))^2);
% 
%     if(isnan(vetorK))
%         vetorK = [1;0;0];
%     end
%     
% %erro da diferença de ângulos desejados versus reais
%     erro_nao_linear = vetorK * theta;
%         
%     TRANSFORMACAO = [cos(estados_medidos(5)), 0, -cos(estados_medidos(4))*sin(estados_medidos(5)); ...
%         0, 1, sin(estados_medidos(4));...
%         sin(estados_medidos(5)), 0, cos(estados_medidos(4))*cos(estados_medidos(5))];  
%     
%     pqrc = TRANSFORMACAO* [(rc(4)-rc_anterior(4))/dt;(rc(5)-rc_anterior(5))/dt;(rc(6)-rc_anterior(6))/dt];
%     
%     u1 = vetor_t' * (R_medido*[0;0;1]); 
%     
%     u2 = cross(estados_medidos(10:12),I*estados_medidos(10:12)) + I*(-Kr*erro_nao_linear + Kw*(pqrc - estados_medidos(10:12)));
% 
% %%%%%FIM CONTROLE NÃO LINEAR%%%%%




    CSI = [CSI 1/2*(trace(eye(3) - R_des'*R_medido))];
    
    erro_nao_linear_anterior = erro_nao_linear;

%calculo do erro quadrático entre os estados desejados e reais de posição e
%orientação
    erro_qd_medio = erro_qd_medio + [(rdes1(1:3,a)-estados(1:3)).^2;0;0;0] + [0;0;0;(rdes1(4:6,a)-estados(4:6)).^2];
    
    if sum(erro_qd_medio)>600
        parar=1;
        erro_qd_medio=1e6*ones(6,1);
    end
    
    if(~isfinite(erro_qd_medio))
        erro = 50000;
        return
    end
    %%%%%%%%%%

    rc_anterior = rc;
    pqrc_anterior = pqrc;
    
%calcula as rotações dos motores
    motor1 = sqrt(u1/(4*k) - u2(2)/(2*k*l) + u2(3)/(4*b));
    motor2 = sqrt(u1/(4*k) + u2(1)/(2*k*l) - u2(3)/(4*b));
    motor3 = sqrt(u1/(4*k) + u2(2)/(2*k*l) + u2(3)/(4*b));
    motor4 = sqrt(u1/(4*k) - u2(1)/(2*k*l) - u2(3)/(4*b));
    
%calcula as forças dos motores
    F1 = u1/4 - u2(2)/(2*l) + u2(3)*k/(4*b);
    F2 = u1/4 + u2(1)/(2*l) - u2(3)*k/(4*b);
    F3 = u1/4 + u2(2)/(2*l) + u2(3)*k/(4*b);
    F4 = u1/4 - u2(1)/(2*l) - u2(3)*k/(4*b);

%     F1 = u1/4 - Iyy*u2(2)/(2*l) + Izz*u2(3)*k/(4*b);
%     F2 = u1/4 + Ixx*u2(1)/(2*l) - Izz*u2(3)*k/(4*b);
%     F3 = u1/4 + Iyy*u2(2)/(2*l) + Izz*u2(3)*k/(4*b);
%     F4 = u1/4 - Ixx*u2(1)/(2*l) - Izz*u2(3)*k/(4*b);

%limita as forças e rotações a valores definidos para cada motor
   forca_Min = k*(rotacaoMin*2*pi/60)^2;

   if F1 < forca_Min
        motor1 = rotacaoMin*(2*pi)/60;
        F1=forca_Min;
    end
    if F2 < forca_Min
        motor2 = rotacaoMin*(2*pi)/60;
        F2=forca_Min;
    end
    if F3 < forca_Min
        motor3 = rotacaoMin*(2*pi)/60;
        F3=forca_Min;
    end
    if F4 < forca_Min
        motor4 = rotacaoMin*(2*pi)/60;
        F4=forca_Min;
    end
    
%    salvar numa matriz a rotacao das 4 helices para cada iteracao.
%     W(a,:) = [motor1 motor1 motor3 motor4];
  
%2pi/60 rad/s = 1 rpm

   forca_Max = k*(rotacaoMax*2*pi/60)^2;
     
   if F1> forca_Max
       F1 = forca_Max;   
       motor1 = rotacaoMax*(2*pi)/60;
   end
    if F2> forca_Max
       F2 = forca_Max;   
       motor2 = rotacaoMax*(2*pi)/60;
    end
    if F3> forca_Max
       F3 = forca_Max;  
       motor3 = rotacaoMax*(2*pi)/60;
    end
    if F4> forca_Max
       F4 = forca_Max; 
       motor4 = rotacaoMax*(2*pi)/60;
    end

    %%--------------------------------PLANTA simula o quadrotor--------------------------------%%
    R = [cos(yaw)*cos(pitch)-sin(roll)*sin(yaw)*sin(pitch), -cos(roll)*sin(yaw), cos(yaw)*sin(pitch)+cos(pitch)*sin(roll)*sin(yaw);...
        cos(pitch)*sin(yaw)+cos(yaw)*sin(roll)*sin(pitch), cos(roll)*cos(yaw), sin(yaw)*sin(pitch)-cos(pitch)*sin(roll)*cos(yaw);...
        -cos(roll)*sin(pitch), sin(roll), cos(roll)*cos(pitch)];
    
      motor = [motor1; motor2; motor3; motor4];
      motor11 = 0.33*motor + 0.67*motor10;
      motor10 = motor11;
      
      F = k * (motor11.^2);
      F1 = F(1); F2 = F(2); F3 = F(3); F4 = F(4);

      %%atualiza o valor do comando u1
     u11 = F1 + F2 + F3 + F4;
    
%%aplica um atraso de primeira ordem no comando u1
%     u11 = 0.33 * u1 + 0.67 * u10;
%     u10 = u11;
    
%       u11 = u1;
    
    epsdd = -g * [0;0;1] + u11/m * R(:,3) - [Ax 0 0;0 Ay 0;0 0 Az]*[vx;vy;vz]/m;
    
    ax = epsdd(1);
    ay = epsdd(2);
    az = epsdd(3);
    
%     if(az < 0 && z <= 0)  %para representar um chão em todo z = 0
%         az = 0;           %
%     end                   %
    
    vx = vx + ax*dt;
    vy = vy + ay*dt;
    vz = vz + az*dt;
    
    x = x + vx*dt;
    y = y + vy*dt;
    zaux = z + vz*dt;
    
    if z>=l && zaux<=l
        z = l;
    else
        if z>=0 && zaux<0
            z = 0;
        else
            z=zaux;
        end
    end
    
%atualiza o valor do comando u2
    u21 = [l*(F2-F4); l*(F3-F1); (F1-F2+F3-F4)*b/k];  
    
%%aplica uma atraso de primeira ordem no comando u2
%     u21 = 0.33*u2 + 0.67*u20;
%     u20 = u21;
    
%     u21 = u2;
    
    wd = inv(I)*(u21 - cross([p;q;r],I*[p;q;r]));

    wax = wd(1);
    way = wd(2);
    waz = wd(3);
    
    p = p+wax*dt;
    q = q+way*dt;
    r = r+waz*dt;
    
    T = [cos(pitch), 0, -cos(roll)*sin(pitch); ...
        0, 1, sin(roll);...
        sin(pitch), 0, cos(roll)*cos(pitch)];    %[p;q;r] = T*(euler rates)
    
    T1=isnan(T);
    s=find(T1==1);
    if sum(s)>0
        parar=1
    end

    eulerd = inv(T) * [p;q;r];
    
    rolld = eulerd(1);
    pitchd = eulerd(2);
    yawd = eulerd(3);
    
    roll = roll + rolld*dt;
    pitch = pitch + pitchd*dt;
    yaw = yaw + yawd*dt;
    
    if(roll > pi) roll = mod(roll,-pi);end
    if(roll < -pi) roll = mod(roll,-pi);end
    
    if(pitch > pi) pitch = mod(pitch,-pi);end
    if(pitch < -pi) pitch = mod(pitch,-pi);end
    
%     if(yaw > pi) yaw = mod(yaw,-pi);end
%     if(yaw < -pi) yaw = mod(yaw,-pi);end
    
    estados = [x;y;z;roll;pitch;yaw;vx;vy;vz;p;q;r];

    %%--------------------------------PLOTS--------------------------------%%
    x_plot = [x_plot x];
    y_plot = [y_plot y];
    z_plot = [z_plot z];
    roll_plot = [roll_plot roll];
    pitch_plot = [pitch_plot pitch];
    yaw_plot = [yaw_plot yaw];
    
    x_des_plot = [x_des_plot rdes1(1,a)];
    y_des_plot = [y_des_plot rdes1(2,a)];
    z_des_plot = [z_des_plot rdes1(3,a)];
    
    x_plot_medido = [x_plot_medido estados_medidos(1)];
    y_plot_medido = [y_plot_medido estados_medidos(2)];
    z_plot_medido = [z_plot_medido estados_medidos(3)];
    roll_plot_medido = [roll_plot_medido estados_medidos(4)];
    pitch_plot_medido = [pitch_plot_medido estados_medidos(5)];
    yaw_plot_medido = [yaw_plot_medido estados_medidos(6)];
    
    b1 = R*braco1 + [x;y;z];
    b2 = R*braco2 + [x;y;z];
    b3 = R*braco3 + [x;y;z];
    b4 = R*braco4 + [x;y;z];
    b5 = R*braco5 + [x;y;z];
    
end

aux = sum([(rdes1(1,:)-mean(rdes1(1,:),2)).^2; (rdes1(2,:) - mean(rdes1(2,:),2)).^2; (rdes1(3,:) - mean(rdes1(3,:),2)).^2; (rdes1(4,:)-mean(rdes1(4,:),2)).^2; (rdes1(5,:) - mean(rdes1(5,:),2)).^2; (rdes1(6,:) - mean(rdes1(6,:),2)).^2],2);

erro = sqrt(erro_qd_medio./aux) * 100;

rplot=[x_plot;y_plot;z_plot;roll_plot;pitch_plot;yaw_plot];

%da mais peso à posição x, y e z e à orientação psi
erro = erro.*[1;1;1;0.01;0.01;1];
% % erro = erro(4:6);

% figure('name','Dados simulados')
% 
% subplot(3,2,1)
% plot(t,x_plot)
% title('X')
% subplot(3,2,3)
% plot(t,y_plot)
% title('Y')
% subplot(3,2,5)
% plot(t,z_plot)
% title('Z')
% subplot(3,2,2)
% plot(t,roll_plot)
% title('ROLL')
% subplot(3,2,4)
% plot(t,pitch_plot)
% title('PITCH')
% subplot(3,2,6)
% plot(t,yaw_plot)
% title('YAW')
% 
% figure('name','Dados simulados assumindo erros de medição')
% 
% subplot(3,2,1)
% plot(t,x_plot_medido)
% title('X')
% subplot(3,2,3)
% plot(t,y_plot_medido)
% title('Y')
% subplot(3,2,5)
% plot(t,z_plot_medido)
% title('Z')
% subplot(3,2,2)
% plot(t,roll_plot_medido)
% title('ROLL')
% subplot(3,2,4)
% plot(t,pitch_plot_medido)
% title('PITCH')
% subplot(3,2,6)
% plot(t,yaw_plot_medido)
% title('YAW')
% 
% figure('name','Comparação entre o desejado e o obtido')
% 
% subplot(3,2,1)
% plot(t,x_plot,'k')
% hold on
% plot(t,rdes(1,:),'k--')
% title('X')
% ylabel('Position (m)')
% xlabel('Time (s)')
% subplot(3,2,3)
% plot(t,y_plot,'k')
% hold on
% plot(t,rdes(2,:),'k--')
% title('Y')
% ylabel('Position (m)')
% xlabel('Time (s)')
% subplot(3,2,5)
% plot(t,z_plot,'k')
% hold on
% plot(t,rdes(3,:),'k--')
% title('Z')
% ylabel('Position (m)')
% xlabel('Time (s)')
% subplot(3,2,2)
% plot(t,roll_plot,'k')
% hold on
% plot(t,rdes(4,:),'k--')
% title('ROLL')
% ylabel('Angle (rad)')
% xlabel('Time (s)')
% subplot(3,2,4)
% plot(t,pitch_plot,'k')
% hold on
% plot(t,rdes(5,:),'k--')
% title('PITCH')
% ylabel('Angle (rad)')
% xlabel('Time (s)')
% subplot(3,2,6)
% plot(t,yaw_plot,'k')
% hold on
% plot(t,rdes(6,:),'k--')
% title('YAW')
% ylabel('Angle (rad)')
% xlabel('Time (s)')
% 
% figure
% plot(t,CSI)
end