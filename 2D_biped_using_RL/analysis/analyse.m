function results = analyse(sln, parameters, to_plot)

% calculate gait quality metrics (distance, step frequency, step length,
% velocity, etc.)

num_steps = length(sln.T);

Y = cat(1,sln.Y{:});

Q = Y(:,1:3);
DQ = Y(:,4:6);

H = zeros(size(Y,1),4);
SWF = zeros(size(Y,1),4);
U = zeros(size(Y,1),2);

time = cat(1,sln.T{:});
time_steps = cat(1,sln.TE{:});
effort = cat(1,sln.TE{:});

dist_step = zeros(num_steps,1);
mean_speed = zeros(num_steps,1);
T_step = zeros(num_steps,1);

time_simulation = sln.T{end}(end);

for i=1:size(Y,1)

    [H(i,1), H(i,2), H(i,3), H(i,4)] = kin_hip(Q(i,:),DQ(i,:));
    
    U(i,:)  = control(0, Q(i,:), DQ(i,:), Q(1,:), DQ(1,:), 0, parameters)';
end

offset_index = 1;

for i = 1:num_steps
    
    [N,~] = size(sln.Y{i});
    offset_index = offset_index + N;
    dist_step(i) = H(offset_index-1,1) - H(offset_index-N,1);
    H_TMP = H((offset_index-N):(offset_index-1),:);
    mean_speed(i) = mean(H_TMP(:,3)); 
    if i > 1
        T_step(i) = sln.T{i}(end) - sln.T{i}(1);
    end

    
end

F_step = T_step.^-1;

effort = (1/(2*time_simulation*max(max(U))))*(sum(sum(U.^2)));
CoT = effort/(sum(dist_step));

max_velocity = max(H(:,3));
min_velociity = min(H(:,3));

results = struct('speed',mean_speed,'dist',(sum(dist_step)),'effort',effort,'CoT',CoT,'Z_hip',H(:,2));

% calculate actuation (you can use the control function)

if to_plot
    % plot the angles
    fig= figure;
    color=[0,0,0];
    set(fig,'defaultAxesColorOrder',[color; color]);
    subplot(3,1,1)
    plot(time,Q(:,1),'b')
    hold on
    xlabel('t')
    ylabel('q1')
    title('q1(t)')
    subplot(3,1,2)
    plot(time,Q(:,2),'b')
    hold on
    xlabel('t')
    ylabel('q2')
    title('q2(t)')
    subplot(3,1,3)
    plot(time,Q(:,3),'b')
    hold on
    xlabel('t')
    ylabel('q3')
    title('q3(t)')
    
    % plot the hip position
    fig= figure;
    color=[0,0,0];
    set(fig,'defaultAxesColorOrder',[color; color]);
    subplot(2,1,1)
    plot(time,H(:,1),'b')
    hold on
    xlabel('t')
    ylabel('x_hip')
    title('x_hip plot')
    subplot(2,1,2)
    plot(time,H(:,2),'b')
    hold on
    xlabel('t')
    ylabel('z_hip')
    title('z_hip plot')

    
    % plot instantaneous and average velocity
    fig= figure;
    color=[0,0,0];
    set(fig,'defaultAxesColorOrder',[color; color]);
    subplot(2,2,1)
    plot(time,H(:,3),'b')
    hold on
    xlabel('t')
    ylabel('dxh')
    title('')
    
    color=[0,0,0];
    set(fig,'defaultAxesColorOrder',[color; color]);
    subplot(2,2,4)
    plot(1:num_steps,dist_step,'b')
    hold on
    xlabel('step number')
    ylabel('lambda')
    title('')
    
    color=[0,0,0];
    set(fig,'defaultAxesColorOrder',[color; color]);
    subplot(2,2,3)
    plot(1:num_steps,F_step,'b')
    hold on
    xlabel('step number')
    ylabel('f')
    title('')
    
    color=[0,0,0];
    set(fig,'defaultAxesColorOrder',[color; color]);
    subplot(2,2,2)
    plot(1:num_steps,mean_speed,'b')
    hold on
    xlabel('step number')
    ylabel('mean dxh')
    title('')
    % plot projections of the limit cycle
    fig= figure;
    color=[0,0,0];
    set(fig,'defaultAxesColorOrder',[color; color]);
    subplot(2,2,1)
    plot(Q(:,1),DQ(:,1),'b')
    hold on
    xlabel('q1')
    ylabel('dq1')
    title('limit cycle q1')
    subplot(2,2,2)
    plot(Q(:,2),DQ(:,2),'b')
    hold on
    xlabel('q2')
    ylabel('dq2')
    title('limit cycle q2')
    subplot(2,2,3)
    plot(Q(:,3),DQ(:,3),'b')
    hold on
    xlabel('q3')
    ylabel('dq3')
    title('limit cycle q3')
    % plot actuation
    fig= figure;
    color=[0,0,0];
    set(fig,'defaultAxesColorOrder',[color; color]);
    subplot(1,2,1)
    plot(time,U(:,1),'b')
    hold on
    xlabel('t')
    ylabel('u1')
    title('u1(t)')
    subplot(1,2,2)
    plot(time,U(:,2),'b')
    hold on
    xlabel('t')
    ylabel('u2')
    title('u2(t)')
    
end

end