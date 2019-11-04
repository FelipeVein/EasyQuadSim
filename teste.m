clear all
clc
Kopt=zeros(1,9);
minimo=[3;3;3;1;1;1;150;20];
%maximo=[100;100;100;100;100;100;1000;100];
maximo=[9;9;9;6;6;6;600;70];
numero_de_ganhos=8;
for i=1:5
    [K,melhor_erro_global]=PSO_novo(numero_de_ganhos,minimo,maximo);
    Kopt=[Kopt;melhor_erro_global K];
end

% K = [4.7266 4.7266 4.7266 2.4436 2.4436 2.4436 3.9468 2.2868];



% clear all
% dt=0.02;
% dtaux = 0.4;
% Ts = 0:dtaux:dtaux*(34-1);
% Ts(end) = Ts(end) - mod(Ts(end),0.02);
% [rdes(:,1),rdv(:,1),rda(:,1),rdj(:,1),rds(:,1)]=Planejamento_toda_trajetoria([0 0.1172 0.4 0.6828 0.9656 1.2486 1.5314 1.8142 1.9314 1.8142 1.5314 1.2486 0.9656 0.6828 0.4 0.1172 0 0.1172 0.4 0.6828 0.9656 1.2486 1.5314 1.8142 1.9314 1.8142 1.5314 1.2486 0.9656 0.6828 0.4 0.1172 0 0],dt,Ts);
% [rdes(:,2),rdv(:,2),rda(:,2),rdj(:,2),rds(:,2)]=Planejamento_toda_trajetoria([0 -0.2828 -0.4 -0.2828 0 0.2828 0.4 0.2828 0 -0.2828 -0.4 -0.2828 0 0.2828 0.4 0.2828 0 -0.2828 -0.4 -0.2828 0 0.2828 0.4 0.2828 0 -0.2828 -0.4 -0.2828 0 0.2828 0.4 0.2828 0 0],dt,Ts);
% [rdes(:,3),rdv(:,3),rda(:,3),rdj(:,3),rds(:,3)]=Planejamento_toda_trajetoria([2 1.5758 1.4 1.5758 2 2.4242 2.6 2.4242 2 1.5758 1.4 1.5758 2 2.4242 2.6 2.4242 2 1.5758 1.4 1.5758 2 2.4242 2.6 2.4242 2 1.5758 1.4 1.5758 2 2.4242 2.6 2.4242 2 2],dt,Ts);
% [rdes(:,6),rdv(:,6),rda(:,6),rdj(:,6),rds(:,6)]=Planejamento_toda_trajetoria([-pi/2 -pi/4 0 pi/4 pi/4 pi/4 0 -pi/4 -pi/2 -3*pi/4 -pi -5*pi/4 -5*pi/4 -5*pi/4 -pi -3*pi/4 -pi/2 -pi/4 0 pi/4 pi/4 pi/4 0 -pi/4 -pi/2 -3*pi/4 -pi -5*pi/4 -5*pi/4 -5*pi/4 -pi -3*pi/4 -pi/2 -pi/2],dt,Ts);
% rdes = rdes';
% rdv = rdv';
% rda = rda';
% rdj = rdj';
% rds = rds';
% orientacao = [deg2rad(178) 0 0]';
% roll = orientacao(1);
% pitch = orientacao(2);
% yaw = orientacao(3);
% R = [cos(yaw)*cos(pitch)-sin(roll)*sin(yaw)*sin(pitch), -cos(roll)*sin(yaw), cos(yaw)*sin(pitch)+cos(pitch)*sin(roll)*sin(yaw);...
% cos(pitch)*sin(yaw)+cos(yaw)*sin(roll)*sin(pitch), cos(roll)*cos(yaw), sin(yaw)*sin(pitch)-cos(pitch)*sin(roll)*cos(yaw);...
% -cos(roll)*sin(pitch), sin(roll), cos(roll)*cos(pitch)];
% K = [4.7266 4.7266 4.7266 2.4436 2.4436 2.4436 3.9468 2.2868];
% [erro,rplot,t1,rdes1] = SimulacaoArtigo(K,dtaux,rdes,rdv,rda,roll,pitch,yaw,R);
% erro'