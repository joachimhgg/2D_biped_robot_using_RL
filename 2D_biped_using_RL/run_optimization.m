clc;
clear;
close all;

%% optimize
% optimize the initial conditions and controller hyper parameters
q0 = [pi/9; -pi/9; 0];
dq0 = [0; 0; 8]; 
x0 = [q0; dq0; control_hyper_parameters()];

% use fminsearch and optimset to control the MaxIter

options = optimset('MaxIter',100);
x0 = fminsearch(@optimziation_fun,x0,options);

%% simulate solution

% extract parameters
q0 = x0(1:3);
dq0 = x0(4:6);
x_opt = x0(7:end);

% simulate
num_steps = 10;
sln = solve_eqns(q0, dq0, num_steps, x_opt);
animate(sln);
results = analyse(sln, x_opt, true);