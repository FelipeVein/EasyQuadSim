function Measures()
    
    global quad;
%     quad.measured_states(1:3,1) = quad.states(1:3) + 1/10*rand(3,1);
%     quad.measured_states(4:6,1) = quad.states(4:6) + 1/50*rand(3,1);
%     quad.measured_states(7:9,1) = quad.states(7:9) + 1/10*rand(3,1);
%     quad.measured_states(10:12,1) = quad.states(10:12) + 1/50 *rand(3,1);

%     quad.measured_states(1:3,1) = quad.states(1:3) + 1/50*rand(3,1)-0.01;
%     quad.measured_states(4:6,1) = quad.states(4:6) + 4*pi/750*rand(3,1)-0.0084;
%     quad.measured_states(7:9,1) = quad.states(7:9) + 1/50*rand(3,1)-0.01;
%     quad.measured_states(10:12,1) = quad.states(10:12) + 4*pi/750*rand(3,1)-0.0084;

    quad.measured_states(1:3,1) = quad.states(1:3);
    quad.measured_states(4:6,1) = quad.states(4:6);
    quad.measured_states(7:9,1) = quad.states(7:9) ;
    quad.measured_states(10:12,1) = quad.states(10:12);

    %doze states x,y,z,roll,pitch,yaw,xdot,ydot,zdot,p,q,r
%     quad.measured_states = quad.states;
end