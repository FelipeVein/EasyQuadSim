function K = PSO()

    % Espaço permitido
%     xmin(1)= 0; xmin(2) = 0; xmin(3) = 0; xmax(1) = 5; xmax(2) = round(5); xmax(3) = round(5); %passo = 0.1;
    xmin=zeros(14,1)+0.001;
    xmax = 10*ones(14,1);
    xmax(13:14) = 100; % = xmax(13:14,1) =100
%     x1 = xmin(1):passo:xmax(1); x2 = xmin(2):passo:xmax(2); x3 = xmin(3):passo:xmax(3); mu = [0;0];
    % FOB
    % FOB = zeros(length(x) , length(y));
    % for i = 1:length(x)
    %     for j = 1:length(y)
    %     X = [x(j);y(i)];
    %     FOB(i,j) = exp(-1/2 * (X -mu)' * (X -mu));
    %     end
    % end
    % melhor_valor_possivel= max(max(FOB));
    % % Plot
    % figure; mesh(x , y , FOB);xlabel('x'); ylabel('y');
    % figure; contour(x , y , FOB);xlabel('x'); ylabel('y');
    % grid on; axis equal;


    %Número de partículas
    n = 100;
    % Posições das partículas
    % (10 - 0.001)*(0<num<1) + 0.001 = 0<VETnum<10
    x1_p = ((xmax(1)-xmin(1)) * rand(1,n) + xmin(1));
    x2_p = ((xmax(2) -xmin(2)) * rand(1,n) + xmin(2));
    x3_p = ((xmax(3) -xmin(3)) * rand(1,n) + xmin(3));
    x4_p = ((xmax(4) -xmin(4)) * rand(1,n) + xmin(4));
    x5_p = ((xmax(5) -xmin(5)) * rand(1,n) + xmin(5));
    x6_p = ((xmax(6) -xmin(6)) * rand(1,n) + xmin(6));
    x7_p = ((xmax(7) -xmin(7)) * rand(1,n) + xmin(7));
    x8_p = ((xmax(8) -xmin(8)) * rand(1,n) + xmin(8));
    x9_p = ((xmax(9) -xmin(9)) * rand(1,n) + xmin(9));
    x10_p = ((xmax(10) -xmin(10)) * rand(1,n) + xmin(10));
    x11_p = ((xmax(11) -xmin(11)) * rand(1,n) + xmin(11));
    x12_p = ((xmax(12) -xmin(12)) * rand(1,n) + xmin(12));
    %(100-0001)*(0<num<1) + 0.001 = 0 < VETnum <100
    x13_p = ((xmax(13) -xmin(13)) * rand(1,n) + xmin(13));
    x14_p = ((xmax(14) -xmin(14)) * rand(1,n) + xmin(14));
    
    % Velocidades das partículas
    v_x1= zeros(1,n);
    v_x2 =zeros(1,n);
    v_x3 =zeros(1,n);
    v_x4 =zeros(1,n);
    v_x5 =zeros(1,n);
    v_x6 =zeros(1,n);
    v_x7 =zeros(1,n);
    v_x8 =zeros(1,n);
    v_x9 =zeros(1,n);
    v_x10 =zeros(1,n);
    v_x11 =zeros(1,n);
    v_x12 =zeros(1,n);
    v_x13 =zeros(1,n);
    v_x14 =zeros(1,n);
    % Plot
    % contour(x , y , FOB);
    % xlabel('x'); ylabel('y'); axis equal; grid on
    % hold on, plot(x_p, y_p, 'ok'), hold off

    % Cálculo do fitness da rodada inicial
    % for i = 1:n
    %     X = [x_p(i) ; y_p(i)];
    for asd = 1:n
        fitness(asd) = 1-sum(SimulacaoArtigo([x1_p(asd) x2_p(asd) x3_p(asd) x4_p(asd) x5_p(asd) x6_p(asd) x7_p(asd) x8_p(asd) x9_p(asd) x10_p(asd) x11_p(asd) x12_p(asd) x13_p(asd) x14_p(asd)]))/6;
    end
    % end
    % A melhor posição do histórico da partícula (x*)
    x1_x_star= x1_p; % no início é a própria posição atual
    x2_x_star= x2_p; % no início é a própria posição atual
    x3_x_star= x3_p; % no início é a própria posição atual
    x4_x_star= x4_p; % no início é a própria posição atual
    x5_x_star= x5_p; % no início é a própria posição atual
    x6_x_star= x6_p; % no início é a própria posição atual
    x7_x_star= x7_p; % no início é a própria posição atual
    x8_x_star= x8_p; % no início é a própria posição atual
    x9_x_star= x9_p; % no início é a própria posição atual
    x10_x_star= x10_p; % no início é a própria posição atual
    x11_x_star= x11_p; % no início é a própria posição atual
    x12_x_star= x12_p; % no início é a própria posição atual
    x13_x_star= x13_p; % no início é a própria posição atual
    x14_x_star= x14_p; % no início é a própria posição atual
    fitness_x_star= fitness; % no início é o fitness atual
    
    % A melhor posição de todos os tempos e de todas as particulas(g*)
    [fitness_g_star, ind_g_star] = max(fitness);
    x1_g_star= x1_p(ind_g_star);
    x2_g_star= x2_p(ind_g_star);
    x3_g_star= x3_p(ind_g_star);
    x4_g_star= x4_p(ind_g_star);
    x5_g_star= x5_p(ind_g_star);
    x6_g_star= x6_p(ind_g_star);
    x7_g_star= x7_p(ind_g_star);
    x8_g_star= x8_p(ind_g_star);
    x9_g_star= x9_p(ind_g_star);
    x10_g_star= x10_p(ind_g_star);
    x11_g_star= x11_p(ind_g_star);
    x12_g_star= x12_p(ind_g_star);
    x13_g_star= x13_p(ind_g_star);
    x14_g_star= x14_p(ind_g_star);
    convergiu_iteracao = 0;
    % % Avaliação do critério de parada do algoritmo
    % erro = melhor_valor_possivel-fitness_g_star;

    % % Plot
    % figure; contour(x , y , FOB); xlabel('x'); ylabel('y');
    % axis equal; grid on; hold on
    % plot(x_p , y_p , 'ok')
    % plot(x_g_star , y_g_star , '*r' , 'linewidth' , 2 , 'markersize' , 10)


    % Inicia o contador de tempo:
    t = 0;
    T = t;
    fit = fitness_g_star;
%     plot(T,fit)
    % Importância da posição g* no cálculo da velocidade:
    alpha = 0.3;
    % Importância da posição x* no cálculo da velocidade:
    beta = 0.3;
    x_star_best = 0;
    % Critério de parada:
    % ksi= 0.01;
    % Loop:
    % while erro > ksi
    while t < 100&& fitness_g_star < 0.95
        % Evolua o tempo:
        t = t + 1
        fitness_g_star
        % Gere as novas velocidades para as partículas:
        e1x1 = rand(size(v_x1)); e2x1 = rand(size(v_x1));
        e1x2 = rand(size(v_x2)); e2x2 = rand(size(v_x2));
        e1x3 = rand(size(v_x3)); e2x3 = rand(size(v_x3));
        e1x4 = rand(size(v_x4)); e2x4 = rand(size(v_x4));
        e1x5 = rand(size(v_x5)); e2x5 = rand(size(v_x5));
        e1x6 = rand(size(v_x6)); e2x6 = rand(size(v_x6));
        e1x7 = rand(size(v_x7)); e2x7 = rand(size(v_x7));
        e1x8 = rand(size(v_x8)); e2x8 = rand(size(v_x8));
        e1x9 = rand(size(v_x9)); e2x9 = rand(size(v_x9));
        e1x10 = rand(size(v_x10)); e2x10 = rand(size(v_x10));
        e1x11 = rand(size(v_x11)); e2x11 = rand(size(v_x11));
        e1x12 = rand(size(v_x12)); e2x12 = rand(size(v_x12));
        e1x13 = rand(size(v_x13)); e2x13 = rand(size(v_x13));
        e1x14 = rand(size(v_x14)); e2x14 = rand(size(v_x14));
        v_x1= (v_x1+alpha* e1x1 .* (x1_g_star-x1_p) + beta * e2x1 .* (x1_x_star-x1_p));
        v_x2= (v_x2+alpha* e1x2 .* (x2_g_star-x2_p) + beta * e2x2 .* (x2_x_star-x2_p));
        v_x3= (v_x3+alpha* e1x3 .* (x3_g_star-x3_p) + beta * e2x3 .* (x3_x_star-x3_p));
        v_x4= (v_x4+alpha* e1x4 .* (x4_g_star-x4_p) + beta * e2x4 .* (x4_x_star-x4_p));
        v_x5= (v_x5+alpha* e1x5 .* (x5_g_star-x5_p) + beta * e2x5 .* (x5_x_star-x5_p));
        v_x6= (v_x6+alpha* e1x6 .* (x6_g_star-x6_p) + beta * e2x6 .* (x6_x_star-x6_p));
        v_x7= (v_x7+alpha* e1x7 .* (x7_g_star-x7_p) + beta * e2x7 .* (x7_x_star-x7_p));
        v_x8= (v_x8+alpha* e1x8 .* (x8_g_star-x8_p) + beta * e2x8 .* (x8_x_star-x8_p));
        v_x9= (v_x9+alpha* e1x9 .* (x9_g_star-x9_p) + beta * e2x9 .* (x9_x_star-x9_p));
        v_x10= (v_x10+alpha* e1x10 .* (x10_g_star-x10_p) + beta * e2x10 .* (x10_x_star-x10_p));
        v_x11= (v_x11+alpha* e1x11 .* (x11_g_star-x11_p) + beta * e2x11 .* (x11_x_star-x11_p));
        v_x12= (v_x12+alpha* e1x12 .* (x12_g_star-x12_p) + beta * e2x12 .* (x12_x_star-x12_p));
        v_x13= (v_x13+alpha* e1x13 .* (x12_g_star-x13_p) + beta * e2x13 .* (x12_x_star-x13_p));
        v_x14= (v_x14+alpha* e1x14 .* (x12_g_star-x14_p) + beta * e2x14 .* (x12_x_star-x14_p));
        % Calcule a nova posição
        x1_p= x1_p+ v_x1;
        x2_p= x2_p+ v_x2;
        x3_p= x3_p+ v_x3;
        x4_p= x4_p+ v_x4;
        x5_p= x5_p+ v_x5;
        x6_p= x6_p+ v_x6;
        x7_p= x7_p+ v_x7;
        x8_p= x8_p+ v_x8;
        x9_p= x9_p+ v_x9;
        x10_p= x10_p+ v_x10;
        x11_p= x11_p+ v_x11;
        x12_p= x12_p+ v_x12;
        x13_p= x13_p+ v_x13;
        x14_p= x14_p+ v_x14;
        
%         x5_p = x1_p;
%         x3_p = x1_p;
%         
%         x4_p = x2_p;
%         x6_p = x2_p;
%         
%         x9_p = x7_p;
%         x8_p = x7_p;
%         
%         x11_p = x10_p;
%         x12_p = x10_p;
        % Evite que as particulassaiam do espaço permitido:
        v_x1(x1_p< xmin(1)) = 0; v_x1(x1_p> xmax(1)) = 0;
        v_x2(x2_p< xmin(2)) = 0; v_x2(x2_p> xmax(2)) = 0;
        v_x3(x3_p< xmin(3)) = 0; v_x3(x3_p> xmax(3)) = 0;
        v_x4(x4_p< xmin(4)) = 0; v_x4(x4_p> xmax(4)) = 0;
        v_x5(x5_p< xmin(5)) = 0; v_x5(x5_p> xmax(5)) = 0;
        v_x6(x6_p< xmin(6)) = 0; v_x6(x6_p> xmax(6)) = 0;
        v_x7(x7_p< xmin(7)) = 0; v_x7(x7_p> xmax(7)) = 0;
        v_x8(x8_p< xmin(8)) = 0; v_x8(x8_p> xmax(8)) = 0;
        v_x9(x9_p< xmin(9)) = 0; v_x9(x9_p> xmax(9)) = 0;
        v_x10(x10_p< xmin(10)) = 0; v_x10(x10_p> xmax(10)) = 0;
        v_x11(x11_p< xmin(11)) = 0; v_x11(x11_p> xmax(11)) = 0;
        v_x12(x12_p< xmin(12)) = 0; v_x12(x12_p> xmax(12)) = 0;
        v_x13(x13_p< xmin(13)) = 0; v_x13(x13_p> xmax(13)) = 0;
        v_x14(x14_p< xmin(14)) = 0; v_x14(x14_p> xmax(14)) = 0;
        x1_p(x1_p< xmin(1)) = xmin(1); x1_p(x1_p> xmax(1)) = xmax(1);
        x2_p(x2_p< xmin(2)) = xmin(2); x2_p(x2_p> xmax(2)) = xmax(2);
        x3_p(x3_p< xmin(3)) = xmin(3); x3_p(x3_p> xmax(3)) = xmax(3);
        x4_p(x4_p< xmin(4)) = xmin(4); x4_p(x4_p> xmax(4)) = xmax(4);
        x5_p(x5_p< xmin(5)) = xmin(5); x5_p(x5_p> xmax(5)) = xmax(5);
        x6_p(x6_p< xmin(6)) = xmin(6); x6_p(x6_p> xmax(6)) = xmax(6);
        x7_p(x7_p< xmin(7)) = xmin(7); x7_p(x7_p> xmax(7)) = xmax(7);
        x8_p(x8_p< xmin(8)) = xmin(8); x8_p(x8_p> xmax(8)) = xmax(8);
        x9_p(x9_p< xmin(9)) = xmin(9); x9_p(x9_p> xmax(9)) = xmax(9);
        x10_p(x10_p< xmin(10)) = xmin(10); x10_p(x10_p> xmax(10)) = xmax(10);
        x11_p(x11_p< xmin(11)) = xmin(11); x11_p(x11_p> xmax(11)) = xmax(11);
        x12_p(x12_p< xmin(12)) = xmin(12); x12_p(x12_p> xmax(12)) = xmax(12);
        x13_p(x13_p< xmin(13)) = xmin(13); x13_p(x13_p> xmax(13)) = xmax(13);
        x14_p(x14_p< xmin(14)) = xmin(14); x14_p(x14_p> xmax(14)) = xmax(14);
        % Calcule o fitness das novas posições:
    %     for i = 1:n
    %         X = [x_p(i) ; y_p(i)];
    
        for asd = 1:n
%             [x1_p(asd) x2_p(asd) x3_p(asd) x4_p(asd) x5_p(asd) x6_p(asd) x7_p(asd) x8_p(asd) x9_p(asd) x10_p(asd) x11_p(asd) x12_p(asd) x13_p(asd) x14_p(asd)]
            fitness(asd) = 1-(sum(SimulacaoArtigo([x1_p(asd) x2_p(asd) x3_p(asd) x4_p(asd) x5_p(asd) x6_p(asd) x7_p(asd) x8_p(asd) x9_p(asd) x10_p(asd) x11_p(asd) x12_p(asd) x13_p(asd) x14_p(asd)]))/6);
        end
        
        ind_star= find(fitness > fitness_x_star);
        % Muda as posições das partículas que melhoraram suas posições
        x1_x_star(ind_star) = x1_p(ind_star);
        x2_x_star(ind_star) = x2_p(ind_star);
        x3_x_star(ind_star) = x3_p(ind_star);
        x4_x_star(ind_star) = x4_p(ind_star);
        x5_x_star(ind_star) = x5_p(ind_star);
        x6_x_star(ind_star) = x6_p(ind_star);
        x7_x_star(ind_star) = x7_p(ind_star);
        x8_x_star(ind_star) = x8_p(ind_star);
        x9_x_star(ind_star) = x9_p(ind_star);
        x10_x_star(ind_star) = x10_p(ind_star);
        x11_x_star(ind_star) = x11_p(ind_star);
        x12_x_star(ind_star) = x12_p(ind_star);
        x13_x_star(ind_star) = x13_p(ind_star);
        x14_x_star(ind_star) = x14_p(ind_star);
        % Muda o fitness das partículas que melhoraram suas posições
        fitness_x_star(ind_star) = fitness(ind_star);
        % Avalie a melhor posição global de todos os tempos (g*):
        % A melhor partícula da rodada
        [x_star_best, ind_star_best] = max(fitness);
        % Se houve melhoria na posição
        if x_star_best> fitness_g_star
            x1_g_star= x1_p(ind_star_best); x2_g_star= x2_p(ind_star_best); x3_g_star= x3_p(ind_star_best);
            x4_g_star= x4_p(ind_star_best); x5_g_star= x5_p(ind_star_best); x6_g_star= x6_p(ind_star_best);
            x7_g_star= x7_p(ind_star_best); x8_g_star= x8_p(ind_star_best); x9_g_star= x9_p(ind_star_best);
            x10_g_star= x10_p(ind_star_best); x11_g_star= x11_p(ind_star_best); x12_g_star= x12_p(ind_star_best);
            x13_g_star= x13_p(ind_star_best); x14_g_star= x14_p(ind_star_best);
            fitness_g_star= x_star_best;
            convergiu_iteracao = t;
        end% Avaliação do critério de parada
    %     erro = melhor_valor_possivel-fitness_g_star;
        % ---------------------
        % Visualização de dados
        % ---------------------
    %     contour(x , y , FOB);
    %     xlabel('x'); ylabel('y'); axis equal; grid on; hold on;
    %     plot(x_p, y_p, 'ok');
    %     plot(x_g_star, y_g_star, '*r', 'linewidth', 2 , 'markersize', 10);
    %     hold off; drawnow;
    %     fprintf('t = %i, x_g* = %.2f, y_g* = %.2f, fitness_g* = %.6f, erro = %.6f \n', t , x_g_star, y_g_star, fitness_g_star, erro);
    %     pause(0.5)
        T = [T t];
        fit = [fit fitness_g_star];
%         plot(T,fit)
%         hold off
%         drawnow
    end

    x1_g_star
    x2_g_star
    x3_g_star
    x4_g_star
    x5_g_star
    x6_g_star
    x7_g_star
    x8_g_star
    x9_g_star
    x10_g_star
    x11_g_star
    x12_g_star
    x13_g_star
    x14_g_star
    fitness_g_star
    convergiu_iteracao
    
    K = [x1_g_star x2_g_star x3_g_star x4_g_star x5_g_star x6_g_star x7_g_star x8_g_star x9_g_star x10_g_star x11_g_star x12_g_star x13_g_star x14_g_star];
end