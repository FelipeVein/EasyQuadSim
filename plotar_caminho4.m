salvar.b1 = [-salvar.l;0;0];
salvar.b2 = [0;-salvar.l;0];
salvar.b3 = [salvar.l;0;0];
salvar.b4 = [0;salvar.l;0];
salvar.b5 = [0;0;salvar.l/2];
writerObj = VideoWriter('Trajetoria3Controle2_branco7.avi'); % Name it.
writerObj.FrameRate = 50; % How many frames per second.
open(writerObj); 
for(a= 1:length(salvar.x_plot))
    if(mod(a,1) == 0)
        posicao = [salvar.x_plot(a);salvar.y_plot(a);salvar.z_plot(a)];
        b1 = salvar.R_plot(:,:,a) * salvar.b1 + posicao;
        b2 = salvar.R_plot(:,:,a) * salvar.b2 + posicao;
        b3 = salvar.R_plot(:,:,a) * salvar.b3 + posicao;
        b4 = salvar.R_plot(:,:,a) * salvar.b4 + posicao;
        b5 = salvar.R_plot(:,:,a) * salvar.b5 + posicao;
        plot3([b1(1) b3(1)],[b1(2) b3(2)],[b1(3) b3(3)],'k','LineWidth',2)
        hold on
        plot3([b2(1) b4(1)],[b2(2) b4(2)],[b2(3) b4(3)],'k','LineWidth',2)
        plot3([posicao(1) b5(1)],[posicao(2) b5(2)],[posicao(3) b5(3)],'k','LineWidth',2)
        desenhar_motor(salvar.l/5,salvar.R_plot(:,:,a),b1,0,1)
        desenhar_motor(salvar.l/5,salvar.R_plot(:,:,a),b2,0,1)
        desenhar_motor(salvar.l/5,salvar.R_plot(:,:,a),b3,1,1)
        desenhar_motor(salvar.l/5,salvar.R_plot(:,:,a),b4,0,1)
        plot3(salvar.x_plot(1:a),salvar.y_plot(1:a),salvar.z_plot(1:a),'k')
        hold on
        plot3(salvar.x_des_plot(1:a),salvar.y_des_plot(1:a),salvar.z_des_plot(1:a),'--r')
        
        plot3(salvar.caminho.X(1:end),salvar.caminho.Y(1:end),salvar.caminho.Z(1:end),'bo')
        for(i = 1:length(salvar.caminho.X))
            text(salvar.caminho.X(i),salvar.caminho.Y(i),salvar.caminho.Z(i)+0.2,mat2str(i))
        end
        grid
        axis([min(salvar.x_plot) max(salvar.x_plot) min(salvar.y_plot) max(salvar.y_plot) min(salvar.z_plot) max(salvar.z_plot)] + [-1 1 -1 1 -1 1] *salvar.l)
%         axis equal
        xlabel('X[m]')
        ylabel('Y[m]')
        zlabel('Z[m]')
        set(gcf,'color','w');
        hold off
        frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
        writeVideo(writerObj, frame);
    end
end
% plot3(salvar.x_plot(1:end),salvar.y_plot(1:end),salvar.z_plot(1:end),'k')
% hold on
% plot3(salvar.x_des_plot(1:end),salvar.y_des_plot(1:end),salvar.z_des_plot(1:end),'--r')
% grid
% % axis equal
% xlabel('X[m]')
% ylabel('Y[m]')
% zlabel('Z[m]')
close(writerObj)