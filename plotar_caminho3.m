load('teste01.mat');
le=length(salvar.armazena_Desejado);

salvar.x_des_plot = salvar.x_des_plot(1:le);
salvar.y_des_plot = salvar.y_des_plot(1:le);
salvar.z_des_plot = salvar.z_des_plot(1:le);
salvar.x_plot = salvar.x_plot(1:le);
salvar.y_plot = salvar.y_plot(1:le);
salvar.z_plot = salvar.z_plot(1:le);

salvar.roll_des_plot = salvar.roll_des_plot(1:le);
salvar.pitch_des_plot = salvar.pitch_des_plot(1:le);
salvar.yaw_des_plot = salvar.yaw_des_plot(1:le);
salvar.roll_plot = salvar.roll_plot(1:le);
salvar.pitch_plot = salvar.pitch_plot(1:le);
salvar.yaw_plot = salvar.yaw_plot(1:le);
    

figure(1)
subplot(2,3,1)
plot(linspace(0,0.02*length(salvar.x_des_plot),length(salvar.x_des_plot)),salvar.x_des_plot,'r--')
hold on
plot(linspace(0,0.02*length(salvar.x_plot),length(salvar.x_plot)),salvar.x_plot,'b')
title('X')
% xlabel('Time [s]')
ylabel('Distance [m]')
grid on

subplot(2,3,2)
plot(linspace(0,0.02*length(salvar.y_des_plot),length(salvar.y_des_plot)),salvar.y_des_plot,'r--')
hold on
plot(linspace(0,0.02*length(salvar.y_plot),length(salvar.y_plot)),salvar.y_plot,'b')
title('Y')
% xlabel('Time [s]')
ylabel('Distance [m]')
grid on

subplot(2,3,3)
plot(linspace(0,0.02*length(salvar.z_des_plot),length(salvar.z_des_plot)),salvar.z_des_plot,'r--')
hold on
plot(linspace(0,0.02*length(salvar.z_plot),length(salvar.z_plot)),salvar.z_plot,'b')
title('Z')
% xlabel('Time [s]')
ylabel('Distance [m]')
grid on

subplot(2,3,4)
plot(linspace(0,0.02*length(salvar.roll_des_plot),length(salvar.roll_des_plot)),salvar.roll_des_plot,'r--')
hold on
plot(linspace(0,0.02*length(salvar.roll_plot),length(salvar.roll_plot)),salvar.roll_plot,'b')
title('Roll (\phi)')
xlabel('Time [s]')
ylabel('Angle [rad]')
grid on

subplot(2,3,5)
plot(linspace(0,0.02*length(salvar.pitch_des_plot),length(salvar.pitch_des_plot)),salvar.pitch_des_plot,'r--')
hold on
plot(linspace(0,0.02*length(salvar.pitch_plot),length(salvar.pitch_plot)),salvar.pitch_plot,'b')
title('Pitch (\theta)')
xlabel('Time [s]')
ylabel('Angle [rad]')
grid on

subplot(2,3,6)
plot(linspace(0,0.02*length(salvar.yaw_des_plot),length(salvar.yaw_des_plot)),salvar.yaw_des_plot,'r--')
hold on
plot(linspace(0,0.02*length(salvar.yaw_plot),length(salvar.yaw_plot)),salvar.yaw_plot,'b')
title('Yaw (\psi)')
xlabel('Time [s]')
ylabel('Angle [rad]')
grid on

set (gcf,'Color','white')
set (gcf,'Position',[10 200 1500 375])