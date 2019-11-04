function ControladorNaoLinear

    global quad;
   %ja tinha sido feito isso em Inicializar_quad 
%     quad.Kcp = quad.K(7:9)';
%     quad.Kcd = quad.K(10:12)';
%     quad.Kr = 9656.655720288503;
%     quad.Kw = 1832.403541405957;

%     quad.Kcp = 91.12*ones(1,3)';
%     quad.Kcd = 18.61*ones(1,3)';
%     quad.Kr = 533.27;
%     quad.Kw = 85.96;
    
    quad.Kcp = 11.79*ones(1,3)';
    quad.Kcd = 7.187*ones(1,3)';
    quad.Kr = 311;
    quad.Kw = 84.35;

    if(quad.iteracao > length(quad.rdes(1,:)))
        Controlador_Position_Hold();
    end
    %recebe ax, ay e az da coluna de iteracao momentanea
    quad.rc = [quad.rda(1:3,quad.iteracao); quad.rdes(4:6,quad.iteracao)];
    
    vetor_t = quad.m*(quad.rc(1:3) + quad.Kcd.*(quad.rdv(1:3,quad.iteracao) - quad.measured_states(7:9)) + quad.Kcp.* (quad.rdes(1:3,quad.iteracao) - quad.measured_states(1:3))) + quad.m*quad.g*[0;0;1];
    
    R_medido = [cos(quad.measured_states(6))*cos(quad.measured_states(5))-sin(quad.measured_states(4))*sin(quad.measured_states(6))*sin(quad.measured_states(5)), -cos(quad.measured_states(4))*sin(quad.measured_states(6)), cos(quad.measured_states(6))*sin(quad.measured_states(5))+cos(quad.measured_states(5))*sin(quad.measured_states(4))*sin(quad.measured_states(6));...
    cos(quad.measured_states(5))*sin(quad.measured_states(6))+cos(quad.measured_states(6))*sin(quad.measured_states(4))*sin(quad.measured_states(5)), cos(quad.measured_states(4))*cos(quad.measured_states(6)), sin(quad.measured_states(6))*sin(quad.measured_states(5))-cos(quad.measured_states(5))*sin(quad.measured_states(4))*cos(quad.measured_states(6));...
    -cos(quad.measured_states(4))*sin(quad.measured_states(5)), sin(quad.measured_states(4)), cos(quad.measured_states(4))*cos(quad.measured_states(5))];
    
    vetor_t_normalizado = vetor_t/norm(vetor_t);
        
    quad.rc(4) = atan((vetor_t_normalizado(1) * sin(quad.rc(6)) - vetor_t_normalizado(2) * cos(quad.rc(6)))/vetor_t_normalizado(3)); %      <- quase certo
    
    quad.rc(5) = atan2(vetor_t_normalizado(1) * cos(quad.rc(6)) + vetor_t_normalizado(2) * sin(quad.rc(6)), vetor_t_normalizado(3)/cos(quad.rc(4)));
    
    if(quad.rc(4) > pi)  quad.rc(4) = mod(quad.rc(4),-pi);end
    if(quad.rc(4) < -pi) quad.rc(4) = mod(quad.rc(4),-pi);end
    
    if(quad.rc(5) > pi)  quad.rc(5) = mod(quad.rc(5),-pi);end
    if(quad.rc(5) < -pi) quad.rc(5) = mod(quad.rc(5),-pi);end
%     
%     if (quad.rc(6)-quad.measured_states(6)) > pi
%         quad.rc(6)=mod(quad.rc(6),-pi);
%     else
%         if (quad.rc(6)-quad.measured_states(6)) > -pi
%             quad.rc(6)=mod(quad.rc(6),-pi);
%         end
%     end
        
    R_des = [cos(quad.rc(6))*cos(quad.rc(5))-sin(quad.rc(4))*sin(quad.rc(6))*sin(quad.rc(5)), -cos(quad.rc(4))*sin(quad.rc(6)), cos(quad.rc(6))*sin(quad.rc(5))+cos(quad.rc(5))*sin(quad.rc(4))*sin(quad.rc(6));...
    cos(quad.rc(5))*sin(quad.rc(6))+cos(quad.rc(6))*sin(quad.rc(4))*sin(quad.rc(5)), cos(quad.rc(4))*cos(quad.rc(6)), sin(quad.rc(6))*sin(quad.rc(5))-cos(quad.rc(5))*sin(quad.rc(4))*cos(quad.rc(6));...
    -cos(quad.rc(4))*sin(quad.rc(5)), sin(quad.rc(4)), cos(quad.rc(4))*cos(quad.rc(5))];

    vetor_t_teste = R_des * [0;0;1];

    if abs( vetor_t_teste(3) - vetor_t_normalizado(3) ) > 0.01
        if quad.rc(4) < -pi
            quad.rc(4) = mod(quad.rc(4),-pi);
        end
        quad.rc(4) = quad.rc(4) - pi;
        quad.rc(5) = atan2(vetor_t_normalizado(1) * cos(quad.rc(6)) + vetor_t_normalizado(2) * sin(quad.rc(6)), vetor_t_normalizado(3)/cos(quad.rc(4)));
    end
    
    quad.CSI = [quad.CSI 1/2*(trace(eye(3) - R_des'*R_medido))];

    quad.rdes(4:5,quad.iteracao) = quad.rc(4:5);
    
    delta_R = R_des'*R_medido;
    
    theta = acos(( trace(delta_R) - 1)/2);
    vetorK(1,1) = (delta_R(3,2) - delta_R(2,3))/sqrt((delta_R(3,2) - delta_R(2,3))^2+(delta_R(1,3) - delta_R(3,1))^2+(delta_R(2,1) - delta_R(1,2))^2);
    vetorK(2,1) = (delta_R(1,3) - delta_R(3,1))/sqrt((delta_R(3,2) - delta_R(2,3))^2+(delta_R(1,3) - delta_R(3,1))^2+(delta_R(2,1) - delta_R(1,2))^2);
    vetorK(3,1) = (delta_R(2,1) - delta_R(1,2))/sqrt((delta_R(3,2) - delta_R(2,3))^2+(delta_R(1,3) - delta_R(3,1))^2+(delta_R(2,1) - delta_R(1,2))^2);
%     vetorKM = ( delta_R - delta_R' ) / ( 2*sin(theta));
%     vetorK(1,1) = vetorKM(3,2);
%     vetorK(2,1) = vetorKM(1,3);
%     vetorK(3,1) = vetorKM(2,1); 
    if(isnan(vetorK))
        vetorK = [1;0;0];
    end
%     waitforbuttonpress
%      produto_vetorial = cross(R_medido*[0;0;1],R_des*[0;0;1])/(norm(R_des*[0;0;1])*norm(R_medido*[0;0;1])); %para só sobrar o sin(theta) multiplicando
    
%     theta2 = asin(norm(produto_vetorial))
    
    %erro_nao_linear = produto_vetorial/norm(produto_vetorial) * theta;
    
    erro_nao_linear = vetorK * theta;
        
    TRANSFORMACAO = [cos(quad.measured_states(5)), 0, -cos(quad.measured_states(4))*sin(quad.measured_states(5)); ...
        0, 1, sin(quad.measured_states(4));...
        sin(quad.measured_states(5)), 0, cos(quad.measured_states(4))*cos(quad.measured_states(5))];  
    
    pqrc = TRANSFORMACAO* [(quad.rc(4)-quad.rc_anterior(4))/quad.dt;(quad.rc(5)-quad.rc_anterior(5))/quad.dt;(quad.rc(6)-quad.rc_anterior(6))/quad.dt];
    
    quad.u1 = vetor_t' * (R_medido*[0;0;1]); 
    
    quad.u2 = cross(quad.measured_states(10:12),quad.I*quad.measured_states(10:12)) + quad.I*(-quad.Kr*erro_nao_linear + quad.Kw*(pqrc - quad.measured_states(10:12)));

%     quad.u2 = cross(quad.measured_states(10:12),quad.I*quad.measured_states(10:12)) + quad.I*(-quad.Kr*erro_nao_linear - quad.Kw*(quad.measured_states(10:12) - R_medido' * R_des * pqrc));
        
    quad.rc_anterior = quad.rc;
    
    if quad.rdes(6,quad.iteracao)> 2*pi
        aux= mod(quad.rdes(6,quad.iteracao),2*pi);
        quad.rdes(6,quad.iteracao) = aux;
    end
    
    quad.x_des_plot = [quad.x_des_plot quad.rdes(1,quad.iteracao)];
    quad.y_des_plot = [quad.y_des_plot quad.rdes(2,quad.iteracao)];
    quad.z_des_plot = [quad.z_des_plot quad.rdes(3,quad.iteracao)];
    quad.roll_des_plot = [quad.roll_des_plot quad.rdes(4,quad.iteracao)];
    quad.pitch_des_plot = [quad.pitch_des_plot quad.rdes(5,quad.iteracao)];
    quad.yaw_des_plot = [quad.yaw_des_plot quad.rdes(6,quad.iteracao)];
    
    
end