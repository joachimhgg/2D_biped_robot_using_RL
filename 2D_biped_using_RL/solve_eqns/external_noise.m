function T_noise = external_noise(t, q, step_number, force_hip)
   
    global t0
    [~, ~, ~, l1, ~, ~, ~] = set_parameters();
    B = eval_B();
    J = [l1*cos(q(1)), 0, 0];
    
    if step_number > 9
        if step_number == 10
            t0=t;
        end
        if t < t0+0.2
            T_noise = J'*force_hip;
    
        else
            T_noise = 0;
        end
    else
        T_noise = 0;
    end
end