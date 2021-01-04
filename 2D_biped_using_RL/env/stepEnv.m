function [NextObs,Reward,IsDone,LoggedSignals] = stepEnv(Actions,LoggedSignals)

    %% Initialisation of the variables for the next time step

    options = odeset('RelTol',1e-5, 'Events', @event_func);
    h = 0.01;
    numstep = 10;

    global sln;
    
    State = LoggedSignals.State;
    y0 = State(1:6);
    t0 = State(7);
    current_step = State(8);
    tspan = linspace(0,h,2);
    
    %% 1 step in the environment
    [T, Y, TE, YE] = ode45(@(t, y) eqnsRL(t, y, y0, current_step,Actions),tspan, y0, options);

    if (isempty(sln.T{current_step}))
        sln.T{current_step} = T(end,:); 
        sln.Y{current_step} = Y(end,:);  
    else 
        sln.T{current_step} = cat(1,sln.T{current_step}, T(end, :));
        sln.Y{current_step}= cat(1,sln.Y{current_step},Y(end, :));
    end 

    %% Impact
    if ~isempty(YE)
        q_m = YE(1:3)';
        dq_m = YE(4:6)';
        [q_p, dq_p] = impact(q_m, dq_m);
        y1 = [q_p; dq_p];
        sln.TE{current_step} = TE;
        sln.YE{current_step} = YE;
        current_step = current_step + 1;
        sln.T{current_step} = [];

    else
        y1 = Y(end,:)';
    end

    
    %% Set up next observation and compute reward

    [Reward,underground,bad_angles] = Rewardfunction(y1,Actions);

    LoggedSignals.State = [y1;T(end);current_step];
    NextObs = y1;
    
    %% Check termination conditions
    
    IsDone = (current_step > numstep);
    
    if (underground || bad_angles)
        IsDone = 1;
    end
    
end