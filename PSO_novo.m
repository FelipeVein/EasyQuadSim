function [K,melhor_erro_global] = PSO_novo(numero_de_ganhos,minimo,maximo)
%numero de ganhos entre minimo < ganho< maximo

%deve- se ter um minimo e maximo para cada ganho 
    dt = 0.02;
    
    if(numero_de_ganhos ~= length(minimo) || numero_de_ganhos ~= length(maximo))
        disp('Erro no tamanho dos vetores ou na quantidade de ganhos')
        return
    end

    %%%% inicializando PSO %%%%
    numero_de_particulas = 625;
    convergiu_iteracao = 0;

    minimo = minimo + 0.00001;
    %posicao da particula na origem inicial, mas na vdd so inicializacao
    posicao_particula = zeros(numero_de_particulas,numero_de_ganhos);
    
    %inserindo posicao inicial das partciulas
    %rand é sempre de 0 a 1
    for i = [1 4 7 8]
        posicao_particula(:,i) = ((maximo(i)-minimo(i)) * rand(numero_de_particulas,1) + minimo(i));
    end
        posicao_particula(:,2)=posicao_particula(:,1);
        posicao_particula(:,3)=posicao_particula(:,1);
        posicao_particula(:,5)=posicao_particula(:,4);
        posicao_particula(:,6)=posicao_particula(:,4);
    
%     posicao_particula(1,:) = [1.5782    1.5782    1.5782    0.8000    0.8000    0.8000 2000 5000.5];

%     dtaux=0.5;
%     Ts=[0:dtaux:dtaux*12];
%     Ts(2:end-1)=Ts(3:end);
%     Ts=Ts(1:end-1);
%     Ts(end)=Ts(end)+mod(Ts(end),dtaux);
%     [rdes(:,1),rdv(:,1),rda(:,1),rdj(:,1),rds(:,1)]=Planejamento_toda_trajetoria([0.00,0.00,0.20,0.40,0.60,0.80,1.00,1.20,1.40,1.60,1.80,1.80],dt,Ts);
%     [rdes(:,2),rdv(:,2),rda(:,2),rdj(:,2),rds(:,2)]=Planejamento_toda_trajetoria([0.00,0.00,0.00,0.40,0.00,-0.40,0.00,0.40,0.00,-0.40,0.00,0.00],dt,Ts);
%     [rdes(:,3),rdv(:,3),rda(:,3),rdj(:,3),rds(:,3)]=Planejamento_toda_trajetoria([0.00,1.00,1.60,1.00,0.40,1.00,1.60,1.00,0.40,1.00,1.60,1.56],dt,Ts);
%     [rdes(:,6),rdv(:,6),rda(:,6),rdj(:,6),rds(:,6)]=Planejamento_toda_trajetoria([0.00,0.00,0.00,pi/2,pi,3*pi/2,2*pi,5*pi/2,3*pi,7*pi/2,4*pi,4*pi],dt,Ts);

%euler angles inicial
roll = 0;
pitch = 0;
yaw = 0;
 
%     %%TRAJETORIA OITO
%     
% dtaux = 0.4;
% Ts = 0:dtaux:dtaux*(34-1);
% Ts(end) = Ts(end) - mod(Ts(end),0.02);
% 
%     [rdes(:,1),rdv(:,1),rda(:,1),rdj(:,1),rds(:,1)]=Planejamento_toda_trajetoria([0 0.1172 0.4 0.6828 0.9656 1.2486 1.5314 1.8142 1.9314 1.8142 1.5314 1.2486 0.9656 0.6828 0.4 0.1172 0 0.1172 0.4 0.6828 0.9656 1.2486 1.5314 1.8142 1.9314 1.8142 1.5314 1.2486 0.9656 0.6828 0.4 0.1172 0 0],dt,Ts);
%     [rdes(:,2),rdv(:,2),rda(:,2),rdj(:,2),rds(:,2)]=Planejamento_toda_trajetoria([0 -0.2828 -0.4 -0.2828 0 0.2828 0.4 0.2828 0 -0.2828 -0.4 -0.2828 0 0.2828 0.4 0.2828 0 -0.2828 -0.4 -0.2828 0 0.2828 0.4 0.2828 0 -0.2828 -0.4 -0.2828 0 0.2828 0.4 0.2828 0 0],dt,Ts);
%     [rdes(:,3),rdv(:,3),rda(:,3),rdj(:,3),rds(:,3)]=Planejamento_toda_trajetoria([2 1.5758 1.4 1.5758 2 2.4242 2.6 2.4242 2 1.5758 1.4 1.5758 2 2.4242 2.6 2.4242 2 1.5758 1.4 1.5758 2 2.4242 2.6 2.4242 2 1.5758 1.4 1.5758 2 2.4242 2.6 2.4242 2 2],dt,Ts);
%     [rdes(:,6),rdv(:,6),rda(:,6),rdj(:,6),rds(:,6)]=Planejamento_toda_trajetoria([-pi/2 -pi/4 0 pi/4 pi/4 pi/4 0 -pi/4 -pi/2 -3*pi/4 -pi -5*pi/4 -5*pi/4 -5*pi/4 -pi -3*pi/4 -pi/2 -pi/4 0 pi/4 pi/4 pi/4 0 -pi/4 -pi/2 -3*pi/4 -pi -5*pi/4 -5*pi/4 -5*pi/4 -pi -3*pi/4 -pi/2 -pi/2],dt,Ts);

%Trajetoria LARS2
 dtaux = 0.6;
    Ts = 0:dtaux:dtaux*12;
    Ts(2:end-1)=Ts(3:end);
    Ts=Ts(1:end-1);
    Ts(end) = Ts(end) - mod(Ts(end),0.02);

    [rdes(:,1),rdv(:,1),rda(:,1),rdj(:,1),rds(:,1)]=Planejamento_toda_trajetoria([-0.40,0.00,0.20,0.40,0.60,0.80,1.00,1.20,1.40,1.60,1.80,1.80],dt,Ts);
    [rdes(:,2),rdv(:,2),rda(:,2),rdj(:,2),rds(:,2)]=Planejamento_toda_trajetoria([0.00,0.00,0.00,0.40,0.00,-0.40,0.00,0.40,0.00,-0.40,0.00,0.00],dt,Ts);
    [rdes(:,3),rdv(:,3),rda(:,3),rdj(:,3),rds(:,3)]=Planejamento_toda_trajetoria([2.00,2.00,2.60,2.00,1.40,2.00,2.60,2.00,1.40,2.00,2.60,2.60],dt,Ts);
    [rdes(:,6),rdv(:,6),rda(:,6),rdj(:,6),rds(:,6)]=Planejamento_toda_trajetoria([0.00,0.00,0.00,pi/2,pi,3*pi/2,2*pi,5*pi/2,3*pi,7*pi/2,4*pi,4*pi],dt,Ts);


    orientacao = [deg2rad(88) 0 0]';
    roll = orientacao(1);
    pitch = orientacao(2);
    yaw = orientacao(3);

    R = [cos(yaw)*cos(pitch)-sin(roll)*sin(yaw)*sin(pitch), -cos(roll)*sin(yaw), cos(yaw)*sin(pitch)+cos(pitch)*sin(roll)*sin(yaw);...
    cos(pitch)*sin(yaw)+cos(yaw)*sin(roll)*sin(pitch), cos(roll)*cos(yaw), sin(yaw)*sin(pitch)-cos(pitch)*sin(roll)*cos(yaw);...
    -cos(roll)*sin(pitch), sin(roll), cos(roll)*cos(pitch)];

    velocidade_particula = zeros(numero_de_particulas,numero_de_ganhos);

    rdes = rdes';
    rdv = rdv';
    rda = rda';
    rdj = rdj';
    rds = rds';  

    erro_atual_da_particula = 10000*ones(625,1);
    
    melhor_posicao_particula = posicao_particula; % no inicio, são iguais
    
    melhor_erro_da_particula = erro_atual_da_particula;
    
    [melhor_erro_da_iteracao, indice_melhor_erro_da_iteracao] = min(melhor_erro_da_particula);
    
    melhor_erro_global = melhor_erro_da_iteracao;
    
    indice_melhor_erro_global = indice_melhor_erro_da_iteracao;
    
    melhor_posicao_particula_global = melhor_posicao_particula(indice_melhor_erro_global,:);
    
    t = 0; %contador de iteracoes
    T = 0; %para plot
    fit = melhor_erro_global; %para plot
    figure
    plot(T,fit,'x-')
    title('Erro do PSO')
    axis([0 2 0 100])
    drawnow
    
    %Parametros do PS0: pesos
    % Importância da posição g* no cálculo da velocidade:
    alpha = 0.1;
    % Importância da posição x* no cálculo da velocidade:
    beta = 0.1;
    
    %100 iteracoes e tolerancia<0.05
    while t < 1 && melhor_erro_global > 0.01
    
        t = t+1;
        
        for i = 1:numero_de_ganhos
        
            velocidade_particula(i,:) = velocidade_particula(i,:) + alpha * rand(1,numero_de_ganhos) .* (melhor_posicao_particula_global - posicao_particula(i,:)) + beta * rand(1,numero_de_ganhos) .* (melhor_posicao_particula(i,:) - posicao_particula(i,:));
    
        end
        
        posicao_particula = posicao_particula + velocidade_particula;
        
        for(i = 1:numero_de_ganhos)
            
            
            %talvez tenho que zerar a velocidade das particulas que
            %chegarma nos limites
           
            %um ganho especifico(alguns dos 12)do vetor de ganhos e testa se algum dos
            %novos ganhos sao maiores q os antigos
            posicao_particula(posicao_particula(:,i) > maximo(i), i) = maximo(i);
            posicao_particula(posicao_particula(:,i) < minimo(i), i) = minimo(i);
        end
        
        posicao_particula(:,2)=posicao_particula(:,1);
        posicao_particula(:,3)=posicao_particula(:,1);
        posicao_particula(:,5)=posicao_particula(:,4);
        posicao_particula(:,6)=posicao_particula(:,4);
        
        for asd = 1:numero_de_particulas
            %ganhos , Y..z de uma particula
            [erro,rplot,t1,rdes1] = SimulacaoArtigo(posicao_particula(asd,:),dtaux,rdes,rdv,rda,roll,pitch,yaw,R);
            erro_atual_da_particula(asd,1) = sum(erro)/6;
        end
        
        indice_particulas_que_melhoraram = find(erro_atual_da_particula < melhor_erro_da_particula);
        
        melhor_posicao_particula(indice_particulas_que_melhoraram,:) = posicao_particula(indice_particulas_que_melhoraram,:);
        melhor_erro_da_particula(indice_particulas_que_melhoraram) = erro_atual_da_particula(indice_particulas_que_melhoraram);
        
        [melhor_erro_da_iteracao, indice_melhor_erro_da_iteracao] = min(melhor_erro_da_particula);
        
        %atualixar o erro global
        if melhor_erro_da_iteracao < melhor_erro_global
            
            convergiu_iteracao = t;
            
            melhor_posicao_particula_global = melhor_posicao_particula(indice_melhor_erro_da_iteracao, :);
            melhor_erro_global = melhor_erro_da_iteracao;
            
        end
        
        T = [T t];
        fit = [fit melhor_erro_global];
        
        plot(T,fit,'x-')
        grid on
        title('Erro do PSO')
        axis([0 2 0 2*mean(fit(2:end))])
        drawnow
    end
    
    convergiu_iteracao
    melhor_erro_global
    K = melhor_posicao_particula_global
end
