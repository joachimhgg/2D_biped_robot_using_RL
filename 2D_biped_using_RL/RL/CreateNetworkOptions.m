% Create DDPG agent and training options for walking robot example
%
% Copyright 2019 The MathWorks, Inc.

%% DDPG Agent Options
agentOptions = rlDDPGAgentOptions;
agentOptions.SampleTime = 0.01;
agentOptions.DiscountFactor = 0.99;
agentOptions.MiniBatchSize = 128;
agentOptions.ExperienceBufferLength = 5e5;
agentOptions.TargetSmoothFactor = 1e-3;
agentOptions.NoiseOptions.MeanAttractionConstant = 5;
agentOptions.NoiseOptions.Variance = 0.5;
agentOptions.NoiseOptions.VarianceDecayRate = 1e-5;

%% Training Options
trainingOptions = rlTrainingOptions;
trainingOptions.MaxEpisodes = 10000;
trainingOptions.MaxStepsPerEpisode = 2000;
trainingOptions.ScoreAveragingWindowLength = 100;
trainingOptions.StopTrainingCriteria = 'AverageReward';
trainingOptions.StopTrainingValue = 300;
trainingOptions.SaveAgentCriteria = 'EpisodeReward';
trainingOptions.SaveAgentValue = 250; % CHANGE THIS VALUE TO SAVE EVERY AGENT ABOVE A CERTAIN REWARD THRESHOLD
trainingOptions.Plots = 'training-progress';
trainingOptions.Verbose = true;
if useParallel
    trainingOptions.Parallelization = 'async';
    trainingOptions.ParallelizationOptions.StepsUntilDataIsSent = 32;
end