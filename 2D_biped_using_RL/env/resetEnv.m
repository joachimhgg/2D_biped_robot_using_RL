function [InitialObservations,LoggedSignals] = resetEnv()

q0 = [pi/9; -pi/9; 0];
dq0 = [0; 0; 8];
t0 = 0;
init_numstep = 1;

global sln;

sln.T = {[]};
sln.Y = {[]};
sln.TE = {[]};
sln.YE = {[]};

LoggedSignals.State = [q0;dq0;t0;init_numstep];
InitialObservations = LoggedSignals.State(1:6);

end