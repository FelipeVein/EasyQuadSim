salvar.b1 = [-salvar.l;0;0];
salvar.b2 = [0;-salvar.l;0];
salvar.b3 = [salvar.l;0;0];
salvar.b4 = [0;salvar.l;0];
salvar.b5 = [0;0;salvar.l/2];
for(i = [1 length(salvar.x_plot)])
posicao = [salvar.x_plot(i);salvar.y_plot(i);salvar.z_plot(i)];
b1 = salvar.R_plot(:,:,i) * salvar.b1 + posicao;
b2 = salvar.R_plot(:,:,i) * salvar.b2 + posicao;
b3 = salvar.R_plot(:,:,i) * salvar.b3 + posicao;
b4 = salvar.R_plot(:,:,i) * salvar.b4 + posicao;
b5 = salvar.R_plot(:,:,i) * salvar.b5 + posicao;
plot3([b1(1) b3(1)],[b1(2) b3(2)],[b1(3) b3(3)],'k','LineWidth',2)
hold on
plot3([b2(1) b4(1)],[b2(2) b4(2)],[b2(3) b4(3)],'k','LineWidth',2)
plot3([posicao(1) b5(1)],[posicao(2) b5(2)],[posicao(3) b5(3)],'k','LineWidth',2)
desenhar_motor(salvar.l/5,salvar.R_plot(:,:,i),b1,0,1)
desenhar_motor(salvar.l/5,salvar.R_plot(:,:,i),b2,0,1)
desenhar_motor(salvar.l/5,salvar.R_plot(:,:,i),b3,1,1)
desenhar_motor(salvar.l/5,salvar.R_plot(:,:,i),b4,0,1)
end
plot3(salvar.x_plot(1:end),salvar.y_plot(1:end),salvar.z_plot(1:end),'k')
hold on
plot3(salvar.x_des_plot(1:end),salvar.y_des_plot(1:end),salvar.z_des_plot(1:end),'--r')
grid
% axis equal
xlabel('X[m]')
ylabel('Y[m]')
zlabel('Z[m]')