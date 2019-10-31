%% rrt reference trajectory
% [result,path] = main_2d_mobile_rrt();
% ref_traj = result.tree(:,path);
% [~,num_steps] = size(path);
% disp(num_steps);


%% hyperbola reference trajectory
% t = log(sqrt(2)-1):1.7629e-04:log(sqrt(2)+1); % 10000 rounds
% x1 = 3*cosh(t) - 3*sqrt(2);
% x2 = 2.5*sinh(t) + 2.5;
% % ref_traj = [x1; x2];
% 
% lqgRT_v2(1, 1e4, 2, 2, 2, [2], eye(2), eye(2), [1 1; 1 -1], eye(2), eye(2), eye(2), 1e-3*eye(2), [0; 0], 1, 1, [0; 5], [0; 2.5], t, [x1; x2], 5);

%% start point, target, and obstacle
% start_point = [0; 0];
% target = 5;
% obs = 2.5;
% 
% syms x1 x2;
% g_U = 1 - (x1)^2 + (x2-obs)^2;
% g_D = 1 - (x1)^2 + (x2-target)^2;


%% test lqg reference tracking with barrier function and sos
% lqgRT(1e-4, num_steps, 2, 2, 2, [2], eye(2), eye(2), [1 1; 1 -1], eye(2), eye(2), eye(2), 1e-3*eye(2), ref_traj, 5, start_point, g_U, g_D)


%% test lqg reference tracking without barrier function and sos
%lqgRT_only(1e-4, num_steps, 2, 2, 2, [2], eye(2), eye(2), [1 1; 1 -1], eye(2), eye(2), eye(2), 1e-3*eye(2), ref_traj, 5, start_point, g_U, g_D)


%% segment reference trajectory
t = -5:(5/(1e4-1)):0;
x1 = t;
x2 = 5*ones(size(t));
lqgRT_v2(1, 1e4, 2, 2, 2, [2], eye(2), eye(2), [1 1; 1 -1], eye(2), eye(2), eye(2), 1e-3*eye(2), [-5; 5], 1, 1, [0; 5], [0; 2.5], t, [x1; x2], 5);