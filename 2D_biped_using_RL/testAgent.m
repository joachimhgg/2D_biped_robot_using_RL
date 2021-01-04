%% Params
close all

global sln;
global Torques; 

global internal_perturbation;
global external_perturbation;

%% Set the value to 1 if you want to add perturbations

internal_perturbation = 0;
external_perturbation = 0;

%% Initialisation of the variable for animate and analyse

sln.T = {};
sln.Y = {};
sln.TE = {};
sln.YE = {};
Torques = []; 

nsteps = 1000; % !! this is the step of the simulation => diff from robot 

%% env Creation 

% Create the observation info
numObs = 6;
ObservationInfo = rlNumericSpec([numObs 1]);
ObservationInfo.Name = 'observations';

% create the action info
numAct = 2;
ActionInfo = rlNumericSpec([numAct 1],'LowerLimit',[-30;-30],'UpperLimit', [30;30]);
ActionInfo.Name = 'foot_torque';

% Environment
env = rlFunctionEnv(ObservationInfo,ActionInfo,'stepEnvTest','resetEnv');

%% Running simulation

simOpts = rlSimulationOptions('MaxSteps',nsteps);
experience = sim(env,saved_agent,simOpts);

animate(sln)
analyseRLTest(sln,Torques(1:end-1,:), 1)
