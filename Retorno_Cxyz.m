function [rdes,rdv,rda,rdj,rds]=Retorno_Cxyz(pontos,dt,vetorT)

% %teste
% pontos=[1 2 4 7];
% dt =0.5;
% vetorT = [1 2 3 4];
if vetorT(1) == 0 
    vetorT(1) = 0;
end
rdes =[];
rdv =[];
rda =[];
rdj =[];
rds =[];
%t:tempos espacados de dt=0.02  e    T:tempos intercalados pelos Waypoints 
syms A T t Funcao_desejada C div
%matriz polinomio e derivadas
A =[0 0 0 0 0 0 0 1;T^7 T^6 T^5 T^4 T^3 T^2 T 1;
    0 0 0 0 0 0 1 0;7*T^6 6*T^5 5*T^4 4*T^3 3*T^2 2*T 1 0;
    0 0 0 0 0 1 0 0;42*T^5 30*T^4 20*T^3 12*T^2 6*T 2 0 0;
    0 0 0 0 1 0 0 0;210*T^4 120*T^3 60*T^2 24*T 6 0 0 0];
    

if length(pontos)==1
    disp('Nao existe trajetoria com apenas um ponto');

  %apenas 2 pontos
elseif length(pontos)==2
    %observe que: vinicial=vfinal = ai = af = ji =jf =0
    vetB = [pontos(1);pontos(2);0;0;0;0;0;0];
    %Calculo de C(matriz dos coeficientes) ainda simbolico
    C = inv(A) * vetB;
    %Conversao de C para numerico e com os valores adequados
    C = double(subs(C,T,vetorT(end)));
    
    %f = c7*t^7 +c6*t^6 +... c0
    Funcao_desejada = 0; 
    for i=1:8
        Funcao_desejada = Funcao_desejada + C(i,1)*t^(8-i)
    end
    
    %obter as derivadas para retornar rdv,rda,rdj,rds
        for w=1:4
            div(w) = diff(Funcao_desejada,w);
        end
    
    %calculo das (trajetorias,vel,a,jerk e snap) desejadas para cada t
    %observe que entrega vetor coluna,pois em CalcTrajetoria recebe coluna
    
    for i=0:dt:vetorT(end)
        rdes= [rdes;double(subs(Funcao_desejada,t,i))];
        rdv = [rdv;double(subs(div(1),t,i))];
        rda = [rda;double(subs(div(2),t,i))];
        rdj = [rdj;double(subs(div(3),t,i))];
        rds = [rds;double(subs(div(4),t,i))];
    end
elseif length(pontos) > 2
  %i : numero de trajetorias
    for i=1:(length(pontos)-1)
        %ponto inicial
        if i==1
            %velocidade em 2:(S2-S1)/(T2-T1).. note q: T1=0
            vetB = [pontos(1);pontos(2);0;(pontos(2)-pontos(1))/(vetorT(2)-vetorT(1));0;0;0;0];
            %ponto final
        elseif i == (length(pontos)-1)
            %Note q: vf=0 e vinicial = (Sinial - Santerior)/Tanterior pois chega
            %nessa velocidade e para(stop) no ultimo ponto
            vetB =[pontos(i);pontos(i+1);(pontos(i)-pontos(i-1))/(vetorT(i)-vetorT(i-1));0;0;0;0;0];
            
            %Pontos intermediarios
        else
            %Vi e Vf ~= 0; Vi = (Satual-S anterior)/(Tatual- Tanterior)
            %Vf = (Sposterior-Satual)/(Tposterior-Tatual)
            vetB = [pontos(i);pontos(i+1);(pontos(i)-pontos(i-1))/(vetorT(i)-vetorT(i-1));(pontos(i+1)-pontos(i))/(vetorT(i+1)-vetorT(i));0;0;0;0];
        end
        %para cada VetB calcula-se um C
        C = inv(A) * vetB;
        
        %C para numerico,substituindo T por Tf-Ti do par de pontos atual
        C = [C double(subs(C,T,vetorT(i+1)-vetorT(i)))];
        
        %Variacao para calcular rdes abaixo (aproveitar o if)
%          k = 0:dt:vetorT(i+1)-vetorT(i);
    end
 t = T(1):dt:T(end);
 i=1; %percorrer o vetor t
 j = 2; %percorrer o vetor T
 flag =1; %objetivo de saber se ja chegou no proximo T marcado pelo usuario,pois tem q mudar os coeficientes C
 %varia te T1 a Tfinal com incremento de 0.02
  while t(i) < t(end)
      while flag==1
          if t(i)<T(j)
              rdes= [rdes sum(C(((j-1)*8)-7:(j-1)*8)'.*[t(i)^7 t(i)^6 t(i)^5 t(i)^4 t(i)^3 t(i)^2 t(i) 1])];
              rdv = [rdv sum(C(((j-1)*8)-7:(j-1)*8)'.*[7*t(i)^6 6*t(i)^5 5*t(i)^4 4*t(i)^3 3*t(i)^2 2*t(i) 1 0])];
              rda = [rda sum(C(((j-1)*8)-7:(j-1)*8)'.*[42*t(i)^5 30*t(i)^4 20*t(i)^3 12*t(i)^2 6*t(i) 2 0 0])];
              rdj = [rdj sum(C(((j-1)*8)-7:(j-1)*8)'.*[210*t(i)^4 120*t(i)^3 60*t(i)^2 24*t(i) 6 0 0 0])];
              rds = [rds sum(C(((j-1)*8)-7:(j-1)*8)'.*[840*t(i)^3 360*t(i)^2 120*t(i) 24 0 0 0 0])];
              i = i +1;   
          else
              flag=0;
          end      
      end
          j = j +1; %temos que trocar os coeficientes
          flag =1; 
  end
%         %f = c7*t^7 +c6*t^6 +... c0
%         Funcao_desejada = 0;
%        % Funcao_desejada = C.*[t^7;t^6;t^5;t^4;t^3;t^2;t;1];
%         for j=1:8
%             Funcao_desejada = Funcao_desejada + C(j,1)*t^(8-j);
%         end
%         %obter as derivadas para retornar rdv,rda,rdj,rds
%         for w=1:4
%             div(w) = diff(Funcao_desejada,w);
%         end
%         %calculo das trajetorias desejadas para cada t
%         %mesmam logica para v,a,j e snap
%         %observe que entrega vetor coluna,pois em CalcTrajetoria recebe coluna
%         for varia=k %varia varia de acordo com o k estabelecido no if acima
%             rdes = [rdes;double(subs(Funcao_desejada,t,varia))];
%             rdv = [rdv;double(subs(div(1),t,varia))];
%             rda = [rda;double(subs(div(2),t,varia))];
%             rdj = [rdj;double(subs(div(3),t,varia))];
%             rds = [rds;double(subs(div(4),t,varia))];
end
   %inserindo valor final
 rdes = [rdes sum(C(end-7:end)'.*[t(i)^7 t(i)^6 t(i)^5 t(i)^4 t(i)^3 t(i)^2 t(i) 1])];
 rdv = [rdv sum(C(end-7:end)'.*[7*t(i)^6 6*t(i)^5 5*t(i)^4 4*t(i)^3 3*t(i)^2 2*t(i) 1 0])];
 rda = [rda sum(C(end-7:end)'.*[42*t(i)^5 30*t(i)^4 20*t(i)^3 12*t(i)^2 6*t(i) 2 0 0])];
 rdj = [rdj sum(C(end-7:end)'.*[210*t(i)^4 120*t(i)^3 60*t(i)^2 24*t(i) 6 0 0 0])];
 rds = [rds sum(C(end-7:end)'.*[840*t(i)^3 360*t(i)^2 120*t(i) 24 0 0 0 0])];
 
%transpondo, pois calcular_trajetoria(funcao chamadora) recebe vetor coluna
 rdes = rdes';
 rdv = rdv';
 rda = rda';
 rdj = rdj';
 rds = rds';
end
 
% figure(10)
% plot(rdes)
% figure(2)
% plot(rdv)
% figure(3)
% plot(rda)
% figure(4)
% plot(rdj)
% figure(5)
% plot(rds)
% drawnow
% waitforbuttonpress
end
       