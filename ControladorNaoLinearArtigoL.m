function ControladorNaoLinearArtigo

    global quad;
    
    %Quem sao esses ganhos: Kp(u1),Kd(u1),Kp(u2),kd(u2) EQ 15a17 e EQ23
    quad.Kcp = 7.51; % 18.1403918162131 (antes de eu mexer)
    quad.Kcd = 4.153;  % 12.6051464768001 (antes de eu mexer)
    quad.Kr = 302.3;  % 10 (antes de eu mexer)
    quad.Kw = 45.55;   %7 (antes de eu mexer)
    
    %Objetivo:manter o Quad parado na ultima posicao e orientacao de Rdes
    %Manter o Quad parado na ultima posicao de Quad.caminho(X,Y,Z,roll,pitch,Yaw)
    if(quad.iteracao > length(quad.rdes(1,:)))
        Controlador_Position_Hold();
    end
    
    %rc =[ax;ay;az] da coluna da iteracao momentanea
    quad.rc = quad.rda(1:3,quad.iteracao); %desejado
    
    
    %equacao 15,16,17 do artigo (igual a Controle Linear)
    %Kcd e o vetor de ganhos derivativos da posicao
    %Kcp é o vetor de ganhos proporcionais da posicao
    quad.rc = quad.rc + quad.Kcd.*(quad.rdv(1:3,quad.iteracao) - [quad.estados_medidos(7:9)]) + quad.Kcp.* (quad.rdes(1:3,quad.iteracao) - quad.estados_medidos(1:3));
    
    %atribuicao de Yaw 
    quad.rc(6) = quad.rdes(6,quad.iteracao);
    
    %era para ser assim nao? So q ele fica parada no lugar
    %vetor_t = Kcd*([quad.rdv(1:3,quad.iteracao)] - [quad.estados_medidos(7:9)]) + Kcp*([quad.rdes(1:3,quad.iteracao)] - [quad.estados_medidos(1:3)]) + quad.m*quad.rc(1:3) + quad.m*quad.g*[0;0;1];
    %incompleto em relacao a equacao 26 do artigo ???? ta certo olha linha
    %24 acima
    vetor_t = quad.m*quad.rc(1:3) + quad.m*quad.g*[0;0;1];
    
    %Yaw =6 , Theta = 5 e phi = 4 :.( CORRETA) como R da eq 1 do artigo,
    %corresponde a R da Eq 41 do artigo:.. deu identidade
    R_medido = [cos(quad.estados_medidos(6))*cos(quad.estados_medidos(5))-sin(quad.estados_medidos(4))*sin(quad.estados_medidos(6))*sin(quad.estados_medidos(5)), -cos(quad.estados_medidos(4))*sin(quad.estados_medidos(6)), cos(quad.estados_medidos(6))*sin(quad.estados_medidos(5))+cos(quad.estados_medidos(5))*sin(quad.estados_medidos(4))*sin(quad.estados_medidos(6));...
    cos(quad.estados_medidos(5))*sin(quad.estados_medidos(6))+cos(quad.estados_medidos(6))*sin(quad.estados_medidos(4))*sin(quad.estados_medidos(5)), cos(quad.estados_medidos(4))*cos(quad.estados_medidos(6)), sin(quad.estados_medidos(6))*sin(quad.estados_medidos(5))-cos(quad.estados_medidos(5))*sin(quad.estados_medidos(4))*cos(quad.estados_medidos(6));...
    -cos(quad.estados_medidos(4))*sin(quad.estados_medidos(5)), sin(quad.estados_medidos(4)), cos(quad.estados_medidos(4))*cos(quad.estados_medidos(5))];
    
    %interesse apenas na direcao, logo normalizou o vetor(EQ 37(bzd))
    vetor_t_normalizado = vetor_t/norm(vetor_t); 
    
    
    %[cos(Yaw);sin(yaw);0] => EQ 36 do artigo  (bxd) vetor coluna 3x1
    b1d = [cos(quad.rc(6));sin(quad.rc(6));0]; 
    
    %EQ 38 artigo (byd) vetor coluna 3x1
    b2d = cross(vetor_t_normalizado, b1d)/norm(cross(vetor_t_normalizado, b1d));
    
    %EQ 39 do artigo identidade
    R_des = [cross(b2d,vetor_t_normalizado), b2d, vetor_t_normalizado];
    
    %obtendo os valores de roll e pitch a partir da matriz acima(EQ 1)
    quad.rc(4) = atan2(R_des(3,2),R_des(2,2)/cos(quad.rc(6)));
    quad.rc(5) = atan2(R_des(1,3)*cos(quad.rc(6))+sin(quad.rc(6))*R_des(2,3), R_des(3,3)/cos(quad.rc(4)));

    quad.rdes(4:6,quad.iteracao) = quad.rc(4:6);
    
    %Eq41 artigo(Correta) Rdes e Rmedido foram atribuidas dentro dessa func
    erro_nao_linear_skew_symmetric = 1/2 * (R_des'*R_medido - R_medido'*R_des);
    %nao verificado ainda 
    erro_nao_linear = [0;0;0];
    erro_nao_linear(1) = erro_nao_linear_skew_symmetric(3,2);
    erro_nao_linear(2) = erro_nao_linear_skew_symmetric(1,3);
    erro_nao_linear(3) = erro_nao_linear_skew_symmetric(2,1);
    
    %matriz T Eq 22 (CORRRETA) identidade
    TRANSFORMACAO = [cos(quad.estados_medidos(5)), 0, -cos(quad.estados_medidos(4))*sin(quad.estados_medidos(5)); 
        0, 1, sin(quad.estados_medidos(4));
        sin(quad.estados_medidos(4)), 0, cos(quad.estados_medidos(4))*cos(quad.estados_medidos(5))];  
    
    %Eq 22 do artigo (Correta)
    quad.pqrc = TRANSFORMACAO* [(quad.rc(4)-quad.rc_anterior(4))/quad.dt;(quad.rc(5)-quad.rc_anterior(5))/quad.dt;(quad.rc(6)-quad.rc_anterior(6))/quad.dt];
    
    %Eq 42 do artigo(Correta se Wdes = quad.pqrc, mas provavel)
    erro_nao_linear_w = quad.estados_medidos(10:12) - R_medido' * R_des * quad.pqrc;
    
    %erro da aceleracao angular
    erro_ac_angular = (quad.pqrc - quad.pqrc_anterior)/quad.dt;
    
    %EQ 27(artigo) CORRETA realmnete é igual a do nao linear
    u1 = vetor_t' * (R_medido*[0;0;1]); 
    
%     u2 = cross(estados_medidos(10:12),I*estados_medidos(10:12)) + (-Kr*erro_nao_linear - Kw * erro_nao_linear_w);
   % ANTES: observe que R medido= R(no artigo e não T)
    u2 = cross(quad.estados_medidos(10:12),quad.I*quad.estados_medidos(10:12)) + quad.I*(-quad.Kr*erro_nao_linear - quad.Kw * erro_nao_linear_w) - quad.I * (cross(quad.estados_medidos(10:12),R_medido'*R_des*quad.pqrc) - R_medido'*R_des*erro_ac_angular);
    %ALTEREI: EQ 43 do artigo
  %  u2 = cross(quad.estados_medidos(10:12),quad.I*quad.estados_medidos(10:12)) + quad.I*(-quad.Kr*erro_nao_linear - quad.Kw * erro_nao_linear_w) - quad.I * (cross(quad.estados_medidos(10:12),TRANSFORMACAO'*R_des*quad.pqrc) - R_medido'*R_des*erro_ac_angular);
    
    
    quad.rc_anterior = quad.rc;
    quad.pqrc_anterior = quad.pqrc;
    
    
    quad.motor(1) = sqrt(u1/(4*quad.k) - quad.Iyy*u2(2)/(2*quad.k*quad.l) + quad.Izz*u2(3)/(4*quad.b));
    quad.motor(2) = sqrt(u1/(4*quad.k) + quad.Ixx*u2(1)/(2*quad.k*quad.l) - quad.Izz*u2(3)/(4*quad.b));
    quad.motor(3) = sqrt(u1/(4*quad.k) + quad.Iyy*u2(2)/(2*quad.k*quad.l) + quad.Izz*u2(3)/(4*quad.b));
    quad.motor(4) = sqrt(u1/(4*quad.k) - quad.Ixx*u2(1)/(2*quad.k*quad.l) - quad.Izz*u2(3)/(4*quad.b));
    
    
    
    quad.x_des_plot = [quad.x_des_plot quad.rdes(1,quad.iteracao)];
    quad.y_des_plot = [quad.y_des_plot quad.rdes(2,quad.iteracao)];
    quad.z_des_plot = [quad.z_des_plot quad.rdes(3,quad.iteracao)];
    quad.roll_des_plot = [quad.roll_des_plot quad.rdes(4,quad.iteracao)];
    quad.pitch_des_plot = [quad.pitch_des_plot quad.rdes(5,quad.iteracao)];
    quad.yaw_des_plot = [quad.yaw_des_plot quad.rdes(6,quad.iteracao)];
    
    
end