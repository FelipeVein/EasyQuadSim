function How_to_write_a_controller()
    
    % First, we need to get the global variable quad
    global quad;
    
    % just add this in the beginning to change to hold position after the trajectory is over
    if(quad.iteracao >= length(quad.rdes(1,:)))
        Controlador_Position_Hold();
    end
     
    
    
% % % % % % % % % % % % % % % % % % % % % % % % % % % % %     
    
%   all important variables are in the global variable quad:
%   quad.iteracao -> current iteration
%   quad.rdes -> matrix with every desired position/orientation (first collumn x, second collumn y, third collumn z, fourth collumn roll, fifth collumn pitch and sixth collumn yaw) 
%   quad.rdv -> matrix with every desired velocity 
%   quad.rda -> matrix with every desired acceleration
%   quad.rdj -> matrix with every desired jerk

%   quad.measured_states -> vector with every measured state. This vector
%   has 12 elements, and they are in the following order: x, y, z, roll,
%   pitch, yaw, dx/dt, dy/dt, dz/dt, p, q, r

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     



%   you should calculate the current measured R matrix like this:

    R_measured = [cos(quad.measured_states(6))*cos(quad.measured_states(5))-sin(quad.measured_states(4))*sin(quad.measured_states(6))*sin(quad.measured_states(5)), -cos(quad.measured_states(4))*sin(quad.measured_states(6)), cos(quad.measured_states(6))*sin(quad.measured_states(5))+cos(quad.measured_states(5))*sin(quad.measured_states(4))*sin(quad.measured_states(6));...
    cos(quad.measured_states(5))*sin(quad.measured_states(6))+cos(quad.measured_states(6))*sin(quad.measured_states(4))*sin(quad.measured_states(5)), cos(quad.measured_states(4))*cos(quad.measured_states(6)), sin(quad.measured_states(6))*sin(quad.measured_states(5))-cos(quad.measured_states(5))*sin(quad.measured_states(4))*cos(quad.measured_states(6));...
    -cos(quad.measured_states(4))*sin(quad.measured_states(5)), sin(quad.measured_states(4)), cos(quad.measured_states(4))*cos(quad.measured_states(5))];
    

%   likewise, you should calculate your desired R matrix:

    R_des = [cos(quad.rc(6))*cos(quad.rc(5))-sin(quad.rc(4))*sin(quad.rc(6))*sin(quad.rc(5)), -cos(quad.rc(4))*sin(quad.rc(6)), cos(quad.rc(6))*sin(quad.rc(5))+cos(quad.rc(5))*sin(quad.rc(4))*sin(quad.rc(6));...
    cos(quad.rc(5))*sin(quad.rc(6))+cos(quad.rc(6))*sin(quad.rc(4))*sin(quad.rc(5)), cos(quad.rc(4))*cos(quad.rc(6)), sin(quad.rc(6))*sin(quad.rc(5))-cos(quad.rc(5))*sin(quad.rc(4))*cos(quad.rc(6));...
    -cos(quad.rc(4))*sin(quad.rc(5)), sin(quad.rc(4)), cos(quad.rc(4))*cos(quad.rc(5))];

%   given that quad.rc is a vector that contains your desired x, y, z, roll, pitch, yaw

    
% % % % % % % % % % % % % % % % % % % % 

%   The output should be:
%   quad.u1 -> a scalar value that represents the input u1
%   quad.u2 -> a 3x1 vector that represents the input u2


% % % % % % % % % % % % % % % % % % % % 

    
    % you can save your current calculations to further use creating new
    % variables like this:
    quad.rc_anterior = quad.rc;
    
    % to enhance yaw plot, your can do this trick
    if quad.rdes(6,quad.iteracao)> 2*pi
        aux= mod(quad.rdes(6,quad.iteracao),2*pi);
        quad.rdes(6,quad.iteracao) = aux;
    end
    
    % after calculating the desired roll and pitch, you need to include this
    % in order to plot, given that quad.rc(4:5) is your desired roll and pitch
    quad.rdes(4:5,quad.iteracao) = quad.rc(4:5);
    
    % You need to include this to plot the results
    quad.x_des_plot = [quad.x_des_plot quad.rdes(1,quad.iteracao)];
    quad.y_des_plot = [quad.y_des_plot quad.rdes(2,quad.iteracao)];
    quad.z_des_plot = [quad.z_des_plot quad.rdes(3,quad.iteracao)];
    quad.roll_des_plot = [quad.roll_des_plot quad.rdes(4,quad.iteracao)];
    quad.pitch_des_plot = [quad.pitch_des_plot quad.rdes(5,quad.iteracao)];
    quad.yaw_des_plot = [quad.yaw_des_plot quad.rdes(6,quad.iteracao)];
    quad.CSI = [quad.CSI 1/2*(trace(eye(3) - R_des'*R_measured))];
    
    
end