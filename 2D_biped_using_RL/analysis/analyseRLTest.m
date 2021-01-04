function results = analyseRLTest(sln,actions, to_plot)

% calculate gait quality metrics (distance, step frequency, step length,
% velocity, etc.)

num_steps = length(sln.Y);

Y = cat(1,sln.Y{:});

Q = Y(:,1:3);
DQ = Y(:,4:6);

H = zeros(size(Y,1),4);
S = zeros(size(Y,1),4);
U = actions; 

time = cat(1,sln.T{:});
size(time) 

dist_step = zeros(num_steps,1);
mean_speed = zeros(num_steps,1);
T_step = zeros(num_steps,1);

time_simulation = time(end);

for i=1:size(Y,1)

    [H(i,1), H(i,2), H(i,3), H(i,4)] = kin_hip(Q(i,:),DQ(i,:));
    [S(i,1), S(i,2), S(i,3), S(i,4)] = kin_swf(Q(i,:),DQ(i,:));

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
    filename = sprintf('q1q2q3.png'); 
     print(gcf,filename,'-dpng','-r300');   
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
    filename = sprintf('x_z_hip.png'); 
     print(gcf,filename,'-dpng','-r300');   
    %plot the hip position
    fig= figure;
    color=[0,0,0];
    set(fig,'defaultAxesColorOrder',[color; color]);
    subplot(2,1,1)
    plot(time,S(:,1),'b')
    hold on
    xlabel('t')
    ylabel('x_swf')
    title('x_swf plot')
    subplot(2,1,2)
    plot(time,S(:,2),'b')
    hold on
    xlabel('t')
    ylabel('z_swf')
    title('z_swf plot')
    filename = sprintf('x_z_swf.png'); 
     print(gcf,filename,'-dpng','-r300');   
    
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
    filename = sprintf('velocityFreq.png'); 
     print(gcf,filename,'-dpng','-r300');   
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
    filename = sprintf('limitCycle.png'); 
     print(gcf,filename,'-dpng','-r300');   
   
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
    filename = sprintf('torques.png'); 
     print(gcf,filename,'-dpng','-r300');   
    
    
end

end