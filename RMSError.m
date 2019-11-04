function RMSError

global quad

tam = length(quad.armazena_Desejado(:,1));
if length(quad.armazena_Medidos(:,1))>length(quad.armazena_Desejado(:,1))
    quad.armazena_Medidos((tam+1):end,:)=[];
end

if length(quad.armazena_Medidos(:,1))<length(quad.armazena_Desejado(:,1))
    quad.armazena_Desejado = quad.armazena_Desejado(1:length(quad.armazena_Medidos));
end

erro_x = sqrt(sum((quad.armazena_Medidos(:,1) - quad.armazena_Desejado(:,1)).^2));
erro_y = sqrt(sum((quad.armazena_Medidos(:,2) - quad.armazena_Desejado(:,2)).^2));
erro_z = sqrt(sum((quad.armazena_Medidos(:,3) - quad.armazena_Desejado(:,3)).^2));
erro_yaw = sqrt(sum((quad.armazena_Medidos(:,4) - quad.armazena_Desejado(:,4)).^2));

x_med = sqrt(sum((quad.armazena_Medidos(:,1) - mean(quad.armazena_Medidos(:,1))).^2));
y_med = sqrt(sum((quad.armazena_Medidos(:,2) - mean(quad.armazena_Medidos(:,2))).^2));
z_med = sqrt(sum((quad.armazena_Medidos(:,3) - mean(quad.armazena_Medidos(:,3))).^2));
yaw_med = sqrt(sum((quad.armazena_Medidos(:,4) - mean(quad.armazena_Medidos(:,4))).^2));

RMSE_X = erro_x*100/x_med;
RMSE_Y = erro_y*100/y_med;
RMSE_Z = erro_z*100/z_med;
RMSE_YAW = erro_yaw*100/yaw_med;

quad.errosRMSE = [RMSE_X RMSE_Y RMSE_Z RMSE_YAW];

