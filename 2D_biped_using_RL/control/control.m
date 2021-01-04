function u = control(t, q, dq, q0, dq0, step_number, parameters)
% You may call control_hyper_parameters and desired_outputs in this function
% you don't necessarily need to use all the inputs to this control function

% extract parameters
kp1 = parameters(1);
kp2 = parameters(2);
kd1 = parameters(3);
kd2 = parameters(4);
alpha = parameters(5);

y1 = q(3) - alpha;
y2 = -q(2) - q(1);
dy1 = dq(3);
dy2 = -dq(2) -dq(1);

% controller for torso
u1 = kp1*y1 + kd1*dy1;

% controller for legs
u2 = kp2*y2 + kd2*dy2; 

% saturate the output torque
u = [u1; u2];
u = max(min(u, 30), -30); 

end