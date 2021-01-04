function [NextObs,Reward,IsDone,LoggedSignals] = stepEnvTest(Actions,LoggedSignals)

    %% initialisation of the variables for the next time step
    
    options = odeset('RelTol',1e-5, 'Events', @event_func);
    h = 0.01;
    numstep = 30; % robot step 

    global sln;
    global Torques; 
    global internal_perturbation;
    
    torques = 30*Actions; 
    torques = max(min(torques, 30), -30); 

    Torques = cat(1,Torques, torques'); 
    
    State = LoggedSignals.State;
    y0 = State(1:6);
    t0 = State(7);
    current_step = State(8);
    tspan = linspace(0,h,2);
    
    %% 1 step in the environment
    
    [T, Y, TE, YE] = ode45(@(t, y) eqnsRLTest(t, y, y0, current_step,Actions),t0+tspan, y0, options);

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
    %% Internal perturbations
    
    if internal_perturbation
        sigma = 1.6;
        noise = normrnd(0,sigma); 
        Noise = [noise;0;0;0;0;0]; 
    else
        Noise = 0;
    end
    
    %% Set up next observation and compute reward
    
    NextObs = y1+Noise;
    
    LoggedSignals.State = [y1;T(end);current_step];
    
    Reward = 0;
    [~,underground,~] = Rewardfunction(y1,Actions); %The agent is not training
    
    %% Check termination conditions
    
    IsDone = (current_step > numstep);
    
    if (underground)
        IsDone = 1;
    end
    
end