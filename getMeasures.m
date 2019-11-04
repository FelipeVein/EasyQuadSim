function measured_states = getMeasures(states)

%     measured_states(1:3,1) = states(1:3) + 1/10*rand(3,1);
%     measured_states(4:6,1) = states(4:6) + 1/50*rand(3,1);
%     measured_states(7:9,1) = states(7:9) + 1/10*rand(3,1);
%     measured_states(10:12,1) = states(10:12) + 1/50 *rand(3,1);
    
    measured_states(1:3,1) = states(1:3) + 1/50*rand(3,1)-0.01;
    measured_states(4:6,1) = states(4:6) + 4*pi/750*rand(3,1)-0.0084;
    measured_states(7:9,1) = states(7:9) + 1/50*rand(3,1)-0.01;
    measured_states(10:12,1) = states(10:12) + 4*pi/750*rand(3,1)-0.0084;

%     measured_states = states;

end