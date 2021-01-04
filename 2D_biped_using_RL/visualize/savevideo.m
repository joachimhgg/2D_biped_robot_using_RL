%%
% This function animates the solution of the equations of motion of the
% three link biped. 
% sln is the solution computed by solve_eqns.m
%%
function savevideo(sln)

figure();
skip = 2;
num_steps = length(sln.Y);
myVideo = VideoWriter('Episode2136'); %open video file
myVideo.FrameRate = 30;  %can adjust this
open(myVideo)
r0 = [0; 0];
for j = 1:num_steps
    Y = sln.Y{j};
    [N, ~] = size(Y);
    for i = 1:skip:N
        q = Y(i, 1:3);
        visualize(q, r0);
        pause(0.01)
        frame = getframe(gcf);
        writeVideo(myVideo,frame);
        hold off
    end
    [x0, ~, ~, ~] = kin_swf(q);
    r0 = r0 + [x0; 0];
end
close(myVideo) %close video file
end