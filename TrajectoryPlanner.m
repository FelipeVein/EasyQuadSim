function [rdes,rdv,rda,rdj,rds]=TrajectoryPlanner(waypoints,dt,T)
t = 0:dt:T(end);

if(length(T) > length(waypoints))
    T = T(1:length(waypoints));
    t = 0:dt:T(end);
end

while(length(waypoints) > length(T))
    T = [T T(2)-T(1)+T(end)];
    t = 0:dt:T(end);
end

MATRIZ = [0  0   0   0   0   0   0   1;
 0  0   0   0   0   0   1   0;
 0  0   0   0   0   2   0   0;
 0  0   0   0   6   0   0   0];

RESULTADO = [waypoints(1);0;0;0];%vetor B 

if(length(waypoints) > 2)
    MATRIZ = [MATRIZ zeros(4,8*(length(waypoints)-2))];
    
    for i=1:(length(waypoints)-2)
        BLOCO(:,:,i) =  [T(i+1)^7 T(i+1)^6 T(i+1)^5 T(i+1)^4 T(i+1)^3 T(i+1)^2 T(i+1) 1   0   0   0   0   0   0   0   0;...
        0  0   0   0   0   0   0   0   T(i+1)^7 T(i+1)^6 T(i+1)^5 T(i+1)^4 T(i+1)^3 T(i+1)^2 T(i+1) 1];
        MATRIZ = [MATRIZ; zeros(2,8*(i-1)) BLOCO(:,:,i) zeros(2,size(MATRIZ,2)-(8*(i+1)))];
        RESULTADO = [RESULTADO;waypoints(i+1);waypoints(i+1)];
    end
    clear BLOCO;
    for i=1:(length(waypoints)-2)
        BLOCO(:,:,i) = [ 7*T(i+1)^6 6*T(i+1)^5 5*T(i+1)^4 4*T(i+1)^3 3*T(i+1)^2 2*T(i+1) 1 0 -7*T(i+1)^6 -6*T(i+1)^5 -5*T(i+1)^4 -4*T(i+1)^3 -3*T(i+1)^2 -2*T(i+1) -1 0;...
 42*T(i+1)^5 30*T(i+1)^4 20*T(i+1)^3 12*T(i+1)^2 6*T(i+1) 2 0 0 -42*T(i+1)^5 -30*T(i+1)^4 -20*T(i+1)^3 -12*T(i+1)^2 -6*T(i+1) -2 0 0;...
 210*T(i+1)^4 120*T(i+1)^3 60*T(i+1)^2 24*T(i+1) 6 0 0 0 -210*T(i+1)^4 -120*T(i+1)^3 -60*T(i+1)^2 -24*T(i+1) -6 0 0 0;...
 840*T(i+1)^3 360*T(i+1)^2 120*T(i+1) 24 0 0 0 0 -840*T(i+1)^3 -360*T(i+1)^2 -120*T(i+1) -24 0 0 0 0;...
 2520*T(i+1)^2 720*T(i+1) 120 0 0 0 0 0 -2520*T(i+1)^2 -720*T(i+1) -120 0 0 0 0 0;...
 5040*T(i+1) 720 0 0 0 0 0 0 -5040*T(i+1) -720 0 0 0 0 0 0];
        MATRIZ = [MATRIZ; zeros(6,8*(i-1)) BLOCO(:,:,i) zeros(6,size(MATRIZ,2)-(8*(i+1)))];
        RESULTADO = [RESULTADO;0;0;0;0;0;0];
    end
    clear BLOCO;
end
BLOCO = [ T(end)^7 T(end)^6 T(end)^5 T(end)^4 T(end)^3 T(end)^2 T(end) 1;...
7*T(end)^6 6*T(end)^5 5*T(end)^4 4*T(end)^3 3*T(end)^2 2*T(end) 1 0;...
42*T(end)^5 30*T(end)^4 20*T(end)^3 12*T(end)^2 6*T(end) 2 0 0;...
210*T(end)^4 120*T(end)^3 60*T(end)^2 24*T(end) 6 0 0 0];
MATRIZ = [MATRIZ; zeros(4,8*(length(waypoints)-2)) BLOCO(:,:)];
RESULTADO = [RESULTADO;waypoints(end);0;0;0];
clear BLOCO;
    

% CONSTANTES = pinv(MATRIZ) * RESULTADO;  % slow calculation.

% faster calculation
[L U P] = lu(MATRIZ);
d = P*RESULTADO;
y = L\d;
CONSTANTES = U\y;

rdes=zeros(1,length(t));
rdv=zeros(1,length(t));
rda=zeros(1,length(t));
rdj=zeros(1,length(t));
rds=zeros(1,length(t));

i = 1;
for j=2:length(waypoints)
    while(1)
        tv=[t(i)^7;t(i)^6;t(i)^5;t(i)^4;t(i)^3;t(i)^2;t(i);1];
        if(t(i) < T(j))
            rdes(:,i)=sum(CONSTANTES(1+(j-2)*8:8+(j-2)*8).*tv);
        else
            break
        end
        tv=[7*t(i)^6 6*t(i)^5 5*t(i)^4 4*t(i)^3 3*t(i)^2 2*t(i) 1 0]';
        if(t(i) < T(j))
            rdv(:,i)=sum(CONSTANTES(1+(j-2)*8:8+(j-2)*8).*tv);
        else
            break
        end
        tv=[42*t(i)^5 30*t(i)^4 20*t(i)^3 12*t(i)^2 6*t(i) 2 0 0]';
        if(t(i) < T(j))
            rda(:,i)=sum(CONSTANTES(1+(j-2)*8:8+(j-2)*8).*tv);
        else
            break
        end
        tv=[210*t(i)^4 120*t(i)^3 60*t(i)^2 24*t(i) 6 0 0 0]';
        if(t(i) < T(j))
            rdj(:,i)=sum(CONSTANTES(1+(j-2)*8:8+(j-2)*8).*tv);
        else
            break
        end
        tv=[840*t(i)^3 360*t(i)^2 120*t(i) 24 0 0 0 0]';
        if(t(i) < T(j))
            rds(:,i)=sum(CONSTANTES(1+(j-2)*8:8+(j-2)*8).*tv);
        else
            break
        end
        i = i+1;
    end
end
rdes(end) = rdes(end-1);
rdv(end) = rdv(end-1);
rda(end) = rda(end-1);
rdj(end) = rdj(end-1);
rds(end) = rds(end-1);
end