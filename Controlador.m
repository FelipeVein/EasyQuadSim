function Controlador

    global quad;
    
    quad.Kcp = 5.501;
    quad.Kcd = 3.386;
    quad.Krollp = 2.08;
    quad.Kpitchp = quad.Krollp;
    quad.Kyawp = quad.Krollp;
    quad.Krolld = 0.5943;
    quad.Kpitchd = quad.Krolld;
    quad.Kyawd = quad.Krolld;
    
    %Objetivo:manter o Quad parado na ultima posicao e orientacao de Rdes
    %iteracoes > colunas da matriz rdes
    if(quad.iteracao >= length(quad.rdes(1,:)))
        %adiciona um coluna para rdes igual a ultima coluna de rdes
        Controlador_Position_Hold();
    end
     
    %posicao = tres linhas(x,y,z) da coluna da iteracao no instante
    %rc = [ax;ay;az;roll,pitch,yaw]
    quad.rc = [quad.rda(1:3,quad.iteracao); quad.rdes(4:6,quad.iteracao)];
    
    %equacao 15,16,17 do artigo
    %Kcd e o vetor de ganhos derivativos do erro de velocidade
    %Kcp é o vetor de ganhos proporcionais do erro de posicao
    quad.rc(1:3) = quad.rc(1:3) + quad.Kcd.*(quad.rdv(1:3,quad.iteracao) - quad.measured_states(7:9)) + quad.Kcp.* (quad.rdes(1:3,quad.iteracao) - quad.measured_states(1:3));
  
    %Equacao 19 20 e 21 do artigo
    quad.rc(4) = 1/quad.g*(quad.rc(1) * sin(quad.rc(6)) - quad.rc(2) *cos(quad.rc(6)));
    quad.rc(5) = 1/quad.g*(quad.rc(1) * cos(quad.rc(6)) + quad.rc(2)*sin(quad.rc(6)));
    
    if quad.rc(4) > pi
        quad.rc(4) = mod(quad.rc(4),-pi);
    end
    if quad.rc(4) < -pi
        quad.rc(4) = mod(quad.rc(4),-pi);
    end
    
    if quad.rc(5) > pi
        quad.rc(5) = mod(quad.rc(5),-pi);
    end
    if quad.rc(5) < -pi
        quad.rc(5) = mod(quad.rc(5),-pi);
    end
    
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

    R_medido = [cos(quad.measured_states(6))*cos(quad.measured_states(5))-sin(quad.measured_states(4))*sin(quad.measured_states(6))*sin(quad.measured_states(5)), -cos(quad.measured_states(4))*sin(quad.measured_states(6)), cos(quad.measured_states(6))*sin(quad.measured_states(5))+cos(quad.measured_states(5))*sin(quad.measured_states(4))*sin(quad.measured_states(6));...
    cos(quad.measured_states(5))*sin(quad.measured_states(6))+cos(quad.measured_states(6))*sin(quad.measured_states(4))*sin(quad.measured_states(5)), cos(quad.measured_states(4))*cos(quad.measured_states(6)), sin(quad.measured_states(6))*sin(quad.measured_states(5))-cos(quad.measured_states(5))*sin(quad.measured_states(4))*cos(quad.measured_states(6));...
    -cos(quad.measured_states(4))*sin(quad.measured_states(5)), sin(quad.measured_states(4)), cos(quad.measured_states(4))*cos(quad.measured_states(5))];
    
    quad.CSI = [quad.CSI 1/2*(trace(eye(3) - R_des'*R_medido))];
    
    %%%%%%%%%%apenas para criar o plot no final
%     rc2 = rda(1:3,iteracao);
%     rc2 = rc2 + Kcd.*(rdv(1:3,iteracao) - [estados(7:9)]) + Kcp.* (rdes(1:3,iteracao) - estados(1:3));
%     
%     rc2(4) = 1/g*(rc2(1) * sin(rdes(6,iteracao)) - rc2(2) *cos(rdes(6,iteracao)));
%     rc2(5) = 1/g*(rc2(1) * cos(rdes(6,iteracao)) + rc2(2)*sin(rdes(6,iteracao)));
%     rc2(6) = rdes(6,iteracao);
    
%     quad.rdes(4:5,quad.iteracao) = quad.rc2(4:5);

% repassando angulos de controle (roll, pitch e yaw)
    quad.rdes(4:5,quad.iteracao) = quad.rc(4:5);

%     erro_qd_medio = erro_qd_medio + (rdes(1:6,iteracao)-estados(1:6)).^2;
%     if(~isfinite(erro_qd_medio))
%         erro = 50000;
%         return
%     end
    %%%%%%%%%%
%     
%     quad.T_medido = [cos(quad.measured_states(5)), 0, -cos(quad.measured_states(4))*sin(quad.measured_states(5));
%         0, 1, sin(quad.measured_states(4));
%         sin(quad.measured_states(5)), 0, cos(quad.measured_states(4))*cos(quad.measured_states(5))];  
   
quad.T_medido = [1 , 0, -quad.measured_states(5); 
        0, 1, (quad.measured_states(4));
        (quad.measured_states(5)), 0, 1];  
%     quad.T_medido = quad.T_medido/det(quad.T_medido);
%   
%equacao 22 onde (rc(4) -rc(4)anterior)/dt = diff ( rc(4) )
%anterior atualizado na linha 78 
    quad.pqrc = quad.T_medido* [(quad.rc(4)-quad.rc_anterior(4))/quad.dt;(quad.rc(5)-quad.rc_anterior(5))/quad.dt;(quad.rc(6)-quad.rc_anterior(6))/quad.dt];


%     T = [cos(pitch), roll, -cos(roll)*sin(pitch); ...
%         roll, 1, sin(roll);...
%         sin(pitch), roll, cos(roll)*cos(pitch)];     %[p;q;r] = T*(euler rates)
    
%     pqrc = rdv(4:6,iteracao);
%     prqc(1) = (rc(4)-rc_anterior(4))/dt;
%     prqc(2) = (rc(5)-rc_anterior(5))/dt;
    
    
    %Equacao 18 artigo CORRETO
    quad.u1 = quad.m*(quad.g+quad.rc(3));%/(cos(quad.measured_states(4))*cos(quad.measured_states(5)));
    
    quad.u2 = [quad.Krollp * (quad.rc(4) - quad.measured_states(4)) + quad.Krolld * (quad.pqrc(1) - quad.measured_states(10));
        quad.Kpitchp * (quad.rc(5) - quad.measured_states(5)) + quad.Kpitchd * (quad.pqrc(2) - quad.measured_states(11));
        quad.Kyawp * (quad.rc(6) - quad.measured_states(6)) + quad.Kyawd * (quad.pqrc(3) - quad.measured_states(12));];
    
    %repassando o valor para calcular na proxima iteracao algumas
    %derivadas(linha 63 eq 22 do artigo)
    quad.rc_anterior = quad.rc;
    
    
    if quad.rdes(6,quad.iteracao)> 2*pi
        aux= mod(quad.rdes(6,quad.iteracao),2*pi);
        quad.rdes(6,quad.iteracao) = aux;
    end
    
    %atualizando a matriz de plotagem da trajetoria otima(desejavel) 
    quad.x_des_plot = [quad.x_des_plot quad.rdes(1,quad.iteracao)];
    quad.y_des_plot = [quad.y_des_plot quad.rdes(2,quad.iteracao)];
    quad.z_des_plot = [quad.z_des_plot quad.rdes(3,quad.iteracao)];
    quad.roll_des_plot = [quad.roll_des_plot quad.rdes(4,quad.iteracao)];
    quad.pitch_des_plot = [quad.pitch_des_plot quad.rdes(5,quad.iteracao)];
    quad.yaw_des_plot = [quad.yaw_des_plot quad.rdes(6,quad.iteracao)];
    
    
end