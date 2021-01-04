function dy = eqnsRLTest(t, y, y0, step_number, action)
% n this is the dimension of the ODE, note that n is 2*DOF, why? 
% y1 = q1, y2 = q2, y3 = q3, y4 = dq1, y5 = dq2, y6 = dq3
% y0 is the states right after impact

%% Initialise parameters

global external_perturbation;

u_scaler = 30; % the network output results between -1 and 1
q = [y(1); y(2); y(3)];
dq = [y(4); y(5); y(6)];

M = eval_M(q);
C = eval_C(q, dq);
G = eval_G(q);
B = eval_B();

%% Compute Network action

u = u_scaler*action;
u = max(min(u, 30), -30);

%% External perturbations

if external_perturbation
    force_hip = -250; 
    T_perturbation = external_noise(t, q, step_number, force_hip);
else
    T_perturbation = 0;
end

%% Compute next state

n = 6;   
dy = zeros(n, 1);
dy(1) = y(4);
dy(2) = y(5);
dy(3) = y(6);
dy(4:6) = M \ (-C*dq - G + B*u + T_perturbation);

end