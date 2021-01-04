% You can set any hyper parameters of the control function here; you may or
% may not want to use the step_number as the input of the function. 
function parameters = control_hyper_parameters(step_number)
kd1 = 77.05;                     
kp1 = 457.5;
kd2 = 5;
kp2 = 161;
% kp1 = 361;
% kp2 = 181.5;
% kd1 = 75.8;
% kd2 = 4;
alpha = 10.4 * pi / 180;
parameters = [kp1, kp2, kd1, kd2, alpha]';
end
