function ControladorNaoLinearArtigo

    global quad;
    
%     quad.Kcp = 52.4;
%     quad.Kcd = 21.28;
%     quad.Kr = 769.52;
%     quad.Kw = 59.83;
    
    quad.Kcp = 13.35;
    quad.Kcd = 3.174;
    quad.Kr = 426.4;
    quad.Kw = 74.76;

%     quad.Kcp = 62.84;
%     quad.Kcd = 17.01;
%     quad.Kr = 406.3;
%     quad.Kw = 51.62;

    if(quad.iteracao > length(quad.rdes(1,:)))
        Controlador_Position_Hold();
    end
    
    %primeira possivel mudanca pelos medidos
    quad.rc(1:3) = quad.rda(1:3,quad.iteracao) + quad.Kcd.*(quad.rdv(1:3,quad.iteracao) - quad.measured_states(7:9)) + quad.Kcp.* (quad.rdes(1:3,quad.iteracao) - quad.measured_states(1:3));
    
    quad.rc(6) = quad.rdes(6,quad.iteracao);
    
    vetor_t = quad.m*quad.rc(1:3) + quad.m*quad.g*[0;0;1];

    quad.R_medido = [cos(quad.measured_states(6))*cos(quad.measured_states(5))-sin(quad.measured_states(4))*sin(quad.measured_states(6))*sin(quad.measured_states(5)), -cos(quad.measured_states(4))*sin(quad.measured_states(6)), cos(quad.measured_states(6))*sin(quad.measured_states(5))+cos(quad.measured_states(5))*sin(quad.measured_states(4))*sin(quad.measured_states(6));...
    cos(quad.measured_states(5))*sin(quad.measured_states(6))+cos(quad.measured_states(6))*sin(quad.measured_states(4))*sin(quad.measured_states(5)), cos(quad.measured_states(4))*cos(quad.measured_states(6)), sin(quad.measured_states(6))*sin(quad.measured_states(5))-cos(quad.measured_states(5))*sin(quad.measured_states(4))*cos(quad.measured_states(6));...
    -cos(quad.measured_states(4))*sin(quad.measured_states(5)), sin(quad.measured_states(4)), cos(quad.measured_states(4))*cos(quad.measured_states(5))];
    
%     quad.Rdesejado =[cos(quad.rdes(6,quad.iteracao))*cos(quad.rdes(5,quad.iteracao))-sin(quad.rdes(4,quad.iteracao))*sin(quad.rdes(6,quad.iteracao))*sin(quad.rdes(5,quad.iteracao)),-cos(quad.rdes(4,quad.iteracao))*sin(quad.rdes(6,quad.iteracao)), cos(quad.rdes(6,quad.iteracao))*sin(quad.rdes(5,quad.iteracao))+cos(quad.rdes(5,quad.iteracao))*sin(quad.rdes(4,quad.iteracao))*sin(quad.rdes(6,quad.iteracao));
%     cos(quad.rdes(5,quad.iteracao))*sin(quad.rdes(6,quad.iteracao))+cos(quad.rdes(6,quad.iteracao))*sin(quad.rdes(4,quad.iteracao))*sin(quad.rdes(5,quad.iteracao)), cos(quad.rdes(4,quad.iteracao))*cos(quad.rdes(6,quad.iteracao)), sin(quad.rdes(6,quad.iteracao))*sin(quad.rdes(5,quad.iteracao))-cos(quad.rdes(5,quad.iteracao))*sin(quad.rdes(4,quad.iteracao))*cos(quad.rdes(6,quad.iteracao));
%     -cos(quad.rdes(4,quad.iteracao))*sin(quad.rdes(5,quad.iteracao)), sin(quad.rdes(4,quad.iteracao)), cos(quad.rdes(4,quad.iteracao))*cos(quad.rdes(5,quad.iteracao))];


    vetor_t_normalizado = vetor_t/norm(vetor_t);
    
    

    b1d = [cos(quad.rc(6));sin(quad.rc(6));0];
    %%Artigo
    b2d = cross(vetor_t_normalizado, b1d)/norm(cross(vetor_t_normalizado, b1d));
%       b2d = cross(vetor_t_normalizado, b1d);

   % R_des = [b1d, b2d, vetor_t_normalizado];
   R_des = [cross(b2d,vetor_t_normalizado), b2d, vetor_t_normalizado];
   
   quad.CSI = [quad.CSI 1/2*(trace(eye(3) - R_des'*quad.R_medido))];
    
    quad.rc(4) = atan2(R_des(3,2),R_des(2,2)/cos(quad.rc(6)));
%     quad.rc(5) = atan2(R_des(1,3)*cos(quad.rc(6))+sin(quad.rc(6))*R_des(2,3), R_des(3,3)/cos(quad.rc(4)));
    quad.rc(5) = atan2(-R_des(3,1)/cos(quad.rc(4)),R_des(3,3)/cos(quad.rc(4)));
    
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
        

    quad.rdes(4:5,quad.iteracao) = quad.rc(4:5);
    
    
    erro_nao_linear_skew_symmetric = 1/2 * (R_des'*quad.R_medido - quad.R_medido'*R_des);
    erro_nao_linear = [0;0;0];
    erro_nao_linear(1) = erro_nao_linear_skew_symmetric(3,2);
    erro_nao_linear(2) = erro_nao_linear_skew_symmetric(1,3);
    erro_nao_linear(3) = erro_nao_linear_skew_symmetric(2,1);
    
    
    TRANSFORMACAO = [cos(quad.measured_states(5)), 0, -cos(quad.measured_states(4))*sin(quad.measured_states(5)); ...
        0, 1, sin(quad.measured_states(4));...
        sin(quad.measured_states(5)), 0, cos(quad.measured_states(4))*cos(quad.measured_states(5))];  
    
%     T1=isnan(TRANSFORMACAO);
%     s=find(T1==1);
%     if sum(s)>0
%         T1 = quad.T10;
%     else
%         T1 = TRANSFORMACAO;
%     end
%     
%     quad.T10 = TRANSFORMACAO;

    T1 = TRANSFORMACAO;
    
    quad.pqrc = T1* [(quad.rc(4)-quad.rc_anterior(4))/quad.dt;(quad.rc(5)-quad.rc_anterior(5))/quad.dt;(quad.rc(6)-quad.rc_anterior(6))/quad.dt];
    
    erro_nao_linear_w = quad.measured_states(10:12) - quad.R_medido' * R_des * quad.pqrc;
    
    
    erro_ac_angular = (quad.pqrc - quad.pqrc_anterior)/quad.dt;
    
    quad.u1 = vetor_t' * (quad.R_medido*[0;0;1]); 
    
   
    quad.u2 = cross(quad.measured_states(10:12),quad.I*quad.measured_states(10:12)) + quad.I * (-quad.Kr*erro_nao_linear - quad.Kw * erro_nao_linear_w) - quad.I * (cross(quad.measured_states(10:12),quad.R_medido'*R_des*quad.pqrc) - quad.R_medido'*R_des*erro_ac_angular);
    
    
    
    quad.rc_anterior = quad.rc;
    quad.pqrc_anterior = quad.pqrc;
    
       
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