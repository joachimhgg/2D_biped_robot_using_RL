function objective_value = optimziation_fun(parameters)

% extract parameters q0, dq0 and x
q0 = parameters(1:3);
dq0 = parameters(4:6);
x = parameters(7:end);

% run simulation
num_steps = 10; % the higher the better, but slow

sln = solve_eqns(q0, dq0, num_steps, x);
results = analyse(sln, x, false);
w1 = 1e-2;
w2 = 1e-3;

% calculate metrics such as distance, mean velocity and cost of transport
max_actuation = 30;
effort = results.effort;
distance = results.dist(end);
velocity = mean(results.speed);
CoT = results.CoT;
objective_value = w1*abs(0.9 - velocity) + w2*CoT;

% handle corner case when model walks backwards (e.g., objective_value =
% 1000)
if(results.dist <= 0)
    objective_value = 1e10;
end

% handle case when model falls (e.g., objective_value = 1000)
if(min(results.Z_hip) <= 0)
    objective_value = 1e10;
end
