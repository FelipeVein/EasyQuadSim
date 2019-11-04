function Model()
    global quad;
    
    roll = quad.orientation(1);
    pitch = quad.orientation(2);
    yaw = quad.orientation(3);
    
    %matriz R(de B para A) CORRETA
    quad.R = [cos(yaw)*cos(pitch)-sin(roll)*sin(yaw)*sin(pitch), -cos(roll)*sin(yaw), cos(yaw)*sin(pitch)+cos(pitch)*sin(roll)*sin(yaw);
        cos(pitch)*sin(yaw)+cos(yaw)*sin(roll)*sin(pitch), cos(roll)*cos(yaw), sin(yaw)*sin(pitch)-cos(pitch)*sin(roll)*cos(yaw);
        -cos(roll)*sin(pitch), sin(roll), cos(roll)*cos(pitch)];

    %EQUACAO5 do artigo considerando a proporcao F =k*w^2 e u1 = F1+..+F4
      % motor(n)=Wn(velocidade angular de rotacao da helice do motor)
      % a(frameA) = quad.linear_acceleration
      %R(:,3) terceira coluna
      %mas parece q ta invertido em relacao a equacao 5 CORRETA
         
    %forcas de cada helice (da neg e positivo depende do sentido de rotacao
%     quad.F(1) = quad.u1/4 - quad.Iyy*quad.u2(2)/(2*quad.l) + quad.Izz*quad.u2(3)*quad.k/(4*quad.b);
%     quad.F(2) = quad.u1/4 + quad.Ixx*quad.u2(1)/(2*quad.l) - quad.Izz*quad.u2(3)*quad.k/(4*quad.b);
%     quad.F(3) = quad.u1/4 + quad.Iyy*quad.u2(2)/(2*quad.l) + quad.Izz*quad.u2(3)*quad.k/(4*quad.b);
%     quad.F(4) = quad.u1/4 - quad.Ixx*quad.u2(1)/(2*quad.l) - quad.Izz*quad.u2(3)*quad.k/(4*quad.b);

    quad.F(1) = quad.u1/4 - quad.u2(2)/(2*quad.l) + quad.u2(3)*quad.k/(4*quad.b);
    quad.F(2) = quad.u1/4 + quad.u2(1)/(2*quad.l) - quad.u2(3)*quad.k/(4*quad.b);
    quad.F(3) = quad.u1/4 + quad.u2(2)/(2*quad.l) + quad.u2(3)*quad.k/(4*quad.b);
    quad.F(4) = quad.u1/4 - quad.u2(1)/(2*quad.l) - quad.u2(3)*quad.k/(4*quad.b);
    
    for i=1:4
        if quad.F(i)<quad.forca_Min quad.F(i)=quad.forca_Min; end
    end
    
    quad.motor(1) = sqrt(quad.F(1)/quad.k);
    quad.motor(2) = sqrt(quad.F(2)/quad.k);
    quad.motor(3) = sqrt(quad.F(3)/quad.k);
    quad.motor(4) = sqrt(quad.F(4)/quad.k);

%armazenando as forcas em cada helice
     quad.Forcas(quad.iteracao,:)=[quad.F(1) quad.F(2) quad.F(3) quad.F(4)]; 

     %salvar numa matriz a rotacao das 4 helices para cada iteracao.
     quad.W(quad.iteracao,:) = [quad.motor(1) quad.motor(2) quad.motor(3) quad.motor(4)];
  
    %W = SQRT(F/K)
    if quad.motor(1) < (quad.rotacaoMin*2*pi/60)
%        quad.motor(1) = sqrt(-quad.F(1)/quad.k);
%         quad.motor(1) = -quad.motor(1);
        quad.motor(1) = quad.rotacaoMin*(2*pi)/60;
        quad.F(1)=quad.forca_Min;
    end
    if quad.motor(2) < (quad.rotacaoMin*2*pi/60)
        %        quad.motor(2) = sqrt(-quad.F(2)/quad.k);
        %         quad.motor(2) = -quad.motor(2);
        quad.motor(2) = quad.rotacaoMin*(2*pi)/60;
        quad.F(2)=quad.forca_Min;
    end
    if quad.motor(3) < (quad.rotacaoMin*2*pi/60)
%        quad.motor(3) = sqrt(-quad.F(3)/quad.k);
%         quad.motor(3) = -quad.motor(3);
        quad.motor(3) = quad.rotacaoMin*(2*pi)/60;
        quad.F(3)=quad.forca_Min;
    end
    if quad.motor(4) < (quad.rotacaoMin*2*pi/60)
%        quad.motor(4) = sqrt(-quad.F(4)/quad.k);
%         quad.motor(4) = -quad.motor(4);
        quad.motor(4) = quad.rotacaoMin*(2*pi)/60;
        quad.F(4)=quad.forca_Min;
    end
    
 quad.forca_Max = quad.k*(quad.rotacaoMax*2*pi/60)^2;
     
    if quad.motor(1) > (quad.rotacaoMax*2*pi/60)
       quad.F(1) = quad.forca_Max;   
       quad.motor(1) = quad.rotacaoMax*(2*pi)/60;
%    elseif quad.F(1)< -quad.forca_Max
%        quad.F(1) = -quad.forca_Max; 
%        quad.motor(1) = -quad.rotacaoMax*(2*pi)/60;
   end
    if quad.motor(2) > (quad.rotacaoMax*2*pi/60)
       quad.F(2) = quad.forca_Max;   
       quad.motor(2) = quad.rotacaoMax*(2*pi)/60;
%    elseif quad.F(2)< -quad.forca_Max
%        quad.F(2) = -quad.forca_Max; 
%        quad.motor(2) = -quad.rotacaoMax*(2*pi)/60;
    end
    if quad.motor(3) > (quad.rotacaoMax*2*pi/60)
       quad.F(3) = quad.forca_Max;  
       quad.motor(3) = quad.rotacaoMax*(2*pi)/60;
%    elseif quad.F(3)< -quad.forca_Max
%        quad.F(3) = -quad.forca_Max;
%        quad.motor(3) = -quad.rotacaoMax*(2*pi)/60;
    end
    if quad.motor(4) > (quad.rotacaoMax*2*pi/60)
       quad.F(4) = quad.forca_Max; 
       quad.motor(4) = quad.rotacaoMax*(2*pi)/60;
%    elseif quad.F(4)< -quad.forca_Max
%        quad.F(4) = -quad.forca_Max; 
%        quad.motor(4) = -quad.rotacaoMax*(2*pi)/60;
   end
    
%       if (quad.motor(1)*60/(2*pi))>quad.rotacaoMax
%         quad.motor(1) = quad.rotacaoMax*(2*pi)/60;
%     elseif quad.motor(1)*60/(2*pi) < -quad.rotacaoMax
%         quad.motor(1) = -quad.rotacaoMax*(2*pi)/60;
%     end
%     
%     if (quad.motor(2)*60/(2*pi))>quad.rotacaoMax
%         quad.motor(2) = quad.rotacaoMax*(2*pi)/60;
%     elseif quad.motor(2)*60/(2*pi) < -quad.rotacaoMax
%         quad.motor(2) = -quad.rotacaoMax*(2*pi)/60;
%     end
%     
%     if (quad.motor(3)*60/(2*pi))>quad.rotacaoMax
%         quad.motor(3) = quad.rotacaoMax*(2*pi)/60;
%     elseif quad.motor(3)*60/(2*pi) < -quad.rotacaoMax
%         quad.motor(3) = -quad.rotacaoMax*(2*pi)/60;
%     end
%    
%     if (quad.motor(4)*60/(2*pi))>quad.rotacaoMax
%         quad.motor(4) = quad.rotacaoMax*(2*pi)/60;
%     elseif quad.motor(4)*60/(2*pi) < -quad.rotacaoMax
%         quad.motor(4) = -quad.rotacaoMax*(2*pi)/60;
%     end

%       quad.Fnovo = 0.33*quad.F + 0.67*quad.Fnovo_anterior;
%       quad.Fnovo_anterior=quad.Fnovo;
%       
    quad.motornovo = 0.33*quad.motor + 0.67*quad.rotor_speed_prev;
    quad.rotor_speed_prev = quad.motornovo;
      
      quad.F = quad.k * (quad.motornovo.^2);
      
    %correcao de rotacao e forca maxima e minima 
    quad.Wnovo(quad.iteracao,:) = [quad.motornovo(1) quad.motornovo(2) quad.motornovo(3) quad.motornovo(4)];
    quad.Forcasnovas(quad.iteracao,:)=[quad.F(1) quad.F(2) quad.F(3) quad.F(4)];    
      
      %substitui pela soma das forcas
      quad.u1novo = quad.F(1) + quad.F(2) + quad.F(3) + quad.F(4);
      %quad.u1 = quad.k*(quad.motornovo(1)^2+quad.motornovo(2)^2+quad.motornovo(3)^2+quad.motornovo(4)^2);
      
%       quad.u1novo = 0.33*quad.u1 + 0.67*quad.u1novo_anterior;
%       quad.u1novo_anterior = quad.u1novo;
      
      %quad.u11(quad.iteracao,1) = quad.u1; era para plotagem
      %ANTIGO
      %quad.linear_acceleration = -quad.g * [0;0;1] + (quad.u1/quad.m)* quad.R(:,3);
      
    
    quad.linear_acceleration = -quad.g * [0;0;1] + (quad.u1novo/quad.m)* quad.R(:,3) - [quad.Ax 0 0;0 quad.Ay 0;0 0 quad.Az]*quad.linear_velocity/quad.m;
    
     %V = V0 +at 
    quad.linear_velocity = quad.linear_velocity + quad.linear_acceleration*quad.dt;
    
    %S=S0 +Vt
    quad.pos_aux = quad.position + quad.linear_velocity*quad.dt;
  
    if quad.position(3)>=quad.l && quad.pos_aux(3)<=quad.l
        quad.position = quad.pos_aux;
        quad.position(3) = quad.l;
    else
        if quad.position(3)>=0 && quad.pos_aux(3)<0
            quad.position = quad.pos_aux;
            quad.position(3) = 0;
        else
            quad.position = quad.pos_aux;
        end
    end
    
    %velocidades angulares no FRAME B(nao fixo)
    p = quad.angular_velocity_quad_frame(1);
    q = quad.angular_velocity_quad_frame(2);
    r = quad.angular_velocity_quad_frame(3);
    
    
    %Equacao 7 artigo
%     quad.u2 = [quad.l*quad.k*(quad.motornovo(2)^2 - quad.motornovo(4)^2);
%           quad.l*quad.k*(quad.motornovo(3)^2 - quad.motornovo(1)^2);
%           quad.b*(quad.motornovo(1)^2 - quad.motornovo(2)^2 + quad.motornovo(3)^2 - quad.motornovo(4)^2)];
    quad.u2novo = [quad.l*(quad.F(2)-quad.F(4));quad.l*(quad.F(3)-quad.F(1));(quad.b/quad.k)*(quad.F(1)-quad.F(2)+quad.F(3)-quad.F(4))];

%     quad.u2novo = 0.33*quad.u2 + 0.67*quad.u2novo_anterior;
%     quad.u2novo_anterior = quad.u2novo;
    %anterior
%     quad.u2novo=quad.u2;
    
    quad.aceleracao_angular_frame_quad = inv(quad.I)*(quad.u2novo - cross(quad.angular_velocity_quad_frame,quad.I*quad.angular_velocity_quad_frame));       
    
%quad.aceleracao_angular_frame_quad = inv(quad.I)*(quad.u2novo - cross(quad.angular_velocity_quad_frame,quad.I*quad.angular_velocity_quad_frame)); 
    
    %refeito acima e abaixo  (linhas 37 e 45)     
%     quad.aceleracao_angular = [(quad.Iyy-quad.Izz) * q*r/quad.Ixx;(quad.Izz-quad.Ixx) * p*r/quad.Iyy;(quad.Ixx-quad.Iyy) * p*q/quad.Izz] + [quad.l*quad.k*(quad.motornovo(2)^2-quad.motornovo(4)^2)/quad.Ixx;quad.l*quad.k*(-quad.motornovo(1)^2+quad.motornovo(3)^2)/quad.Iyy;quad.b*(quad.motornovo(1)^2-quad.motornovo(2)^2+quad.motornovo(3)^2-quad.motornovo(4)^2)/quad.Izz];
%     quad.angular_velocity_quad_frame = quad.angular_velocity_quad_frame + quad.aceleracao_angular*quad.dt;
   
    %w = w0 + alfa*t (n deveria ser aceleracao no frame B?)
    %PAR DA LINHA 37
    quad.angular_velocity_quad_frame = quad.angular_velocity_quad_frame + quad.aceleracao_angular_frame_quad*quad.dt;
   
    %matriz que relaciona p,q,r com os angulos de euler no FRAME A
    %[variacao dos angulos no frame A] = T-¹* w(vel angular no frame B =(p,q,r)) 
    quad.T = [cos(pitch), 0, -cos(roll)*sin(pitch);
        0, 1, sin(roll);
        sin(pitch), 0, cos(roll)*cos(pitch)];     %[p;q;r] = T*(euler rates)
    
    %[variacao dos angulos em A] = T-¹* w(vel angular no frame B =(p,q,r))
    %ndot = T^-1 * [p;q;r] Equacao 8 CORRETA
    quad.velocidade_angular_frame_inercial = inv(quad.T) * quad.angular_velocity_quad_frame;
    
    %orientation é VETOR PHI,ROLL, YAW ??? Sim
    %Theta = theta_zero +wt
    quad.orientation = quad.orientation + quad.velocidade_angular_frame_inercial*quad.dt;
    
    %QUAL O OBJETIVO AQUI? limitar os angulos roll e pitch entre pi e -pi
    if(quad.orientation(1) > pi) quad.orientation(1) = mod(quad.orientation(1),-pi);end
    if(quad.orientation(1) < -pi) quad.orientation(1) = mod(quad.orientation(1),-pi);end
    
    if(quad.orientation(2) > pi) quad.orientation(2) = mod(quad.orientation(2),-pi);end
    if(quad.orientation(2) < -pi) quad.orientation(2) = mod(quad.orientation(2),-pi);end

    %ALTERACAO DO CODIGO:para manter Yaw entre -pi e pi
%     if(quad.orientation(3) > pi) quad.orientation(3) = mod(quad.orientation(3),-pi);end
%     if(quad.orientation(3) < -pi) quad.orientation(3) = mod(quad.orientation(3),-pi);end
    %x y z roll pitch yaw vx, vy,vz   p q r (12 estados)
    if quad.orientation(3)> 2*pi
        aux=mod(quad.orientation(3),2*pi);
    else
        aux=quad.orientation(3);
    end
    
    quad.estados = [quad.position;quad.orientation;quad.linear_velocity;quad.angular_velocity_quad_frame];
    
    %     quad.erro_qd_medio = quad.erro_qd_medio + [(rdes(1:3,a)-estados(1:3)).^2;0;0;0] + [0;0;0;(cos(rdes(4:6,a))-cos(estados(4:6))).^2];

    quad.armazena_Medidos(quad.iteracao,:) = [quad.position' quad.orientation(3)];
   
    %OBJETIVO: acrescentar as variaveis obtidas nessa iteracao no plot
    quad.x_plot = [quad.x_plot quad.position(1)];
    quad.y_plot = [quad.y_plot quad.position(2)];
    quad.z_plot = [quad.z_plot quad.position(3)];
    quad.roll_plot = [quad.roll_plot quad.orientation(1)];
    quad.pitch_plot = [quad.pitch_plot quad.orientation(2)];
    quad.yaw_plot = [quad.yaw_plot aux];
    %Como assim ? era teste do filipe
    quad.R_plot(:,:,quad.iteracao) = quad.R;
    
end