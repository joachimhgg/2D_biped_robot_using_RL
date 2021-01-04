% Walking Robot -- DDPG Agent Training Script (2D)
% Copyright 2019 The MathWorks, Inc.

%% SET UP ENVIRONMENT
% Speedup options
useFastRestart = true;
useGPU = false;
useParallel = false;

global sln;

sln.T = {};
sln.Y = {};
sln.TE = {};
sln.YE = {};

q0 = [pi/9; -pi/9; 0];
dq0 = [0; 0; 0];

t0 = 0;
init_numstep = 1;

% Create the observation info
numObs = 6;
ObservationInfo = rlNumericSpec([numObs 1]);
ObservationInfo.Name = 'observations';

% create the action info
numAct = 2;
ActionInfo = rlNumericSpec([numAct 1],'LowerLimit',[-30;-30],'UpperLimit', [30;30]);
ActionInfo.Name = 'foot_torque';

% Environment
env = rlFunctionEnv(ObservationInfo,ActionInfo,'stepEnv','resetEnv');

%% CREATE NEURAL NETWORKS
CreateNetwork;
                     
%% CREATE AND TRAIN AGENT
CreateNetworkOptions;
agent = rlDDPGAgent(actor,critic,agentOptions);

trainingResults = train(agent,env,trainingOptions);

%% SAVE AGENT
reset(agent); % Clears the experience buffer
curDir = pwd;
saveDir = 'savedAgents';
cd(saveDir)
save(['trainedAgent_2D_' datestr(now,'mm_DD_YYYY_HHMM')],'agent');
save(['trainingResults_2D_' datestr(now,'mm_DD_YYYY_HHMM')],'trainingResults');
cd(curDir)