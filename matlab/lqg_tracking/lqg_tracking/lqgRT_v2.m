function [] = lqgRT_v2(T, num_steps, n, m, p, pMinusS, A, B, C, Sigma_w, Sigma_v, Q, R, start_point, rd_tar, rd_obs, target, obs, t, parametric_func, degree)

%%% T              -- the final time of the system
%%% dt             -- the time duration of each iteration
%%% num_steps      -- the number of iterations during T, 1000 is not enough
%%% n              -- the dimension of the system state
%%% m              -- the dimension of the system input
%%% p              -- the dimension of the system observation
%%% pMinusS        -- the index list of the secure sensors(row vector)
%%% A              -- n*n state matrix
%%% B              -- n*m input matrix
%%% C              -- p*n output matrix
%%% C_alpha        -- the matrix obtained by selecting the rows from C indexed in
%%%                   the observation matrix y that are not affected by the adversary
%%% Q              -- n*n cost matrix
%%% R              -- m*m cost matrix
%%% Sigma_w        -- n*n autocorrelation matrix
%%% Sigma_v        -- p*p autocorrelation matrix
%%% Sigma_v_alpha  -- the covariance matrix of v_alpha
%%% ref_traj       -- n*num_steps polynomial reference trajectory
%%% degree         -- the degree of the polyfit for the reference trajectory

%clear all
%close all
%clc

dt = T/num_steps;

for i = 1:n
    eval(sprintf('syms x%d', i));
end

g_D = rd_tar^2;
g_U = rd_obs^2;
for i = 1:n
    eval(sprintf('g_D = g_D - (x%d - target(i))^2', i));
    eval(sprintf('g_U = g_U - (x%d - obs(i))^2', i));
end

ref_traj = parametric_func;


%% reference trajectory enlargement %%
disp(num_steps);
if num_steps <= 1000
    disp(num_steps);
    temp_ref = ref_traj;
    %[~,col_ref_traj] = size(ref_traj);
    %ref_traj = [ref_traj zeros(n,(num_steps*99 + 1 - col_ref_traj))];
    real_ref = zeros(n,((num_steps - 1)*99 + 1));
    disp(size(real_ref));
    % for i = 1:1:(num_steps-1)
    %     for j = 1:1:n
    %         tmp = linspace(temp(j,i),temp(j,i+1));
    %         ref_traj(j,:) = [ref_traj(j,1:end-1) tmp];
    %     end
    % end

    for i = 1:1:n
        tmp = double.empty(1,0);
        for j = 1:1:(num_steps-1)

            tmp = [tmp(1:(end-1)) linspace(temp_ref(i,j),temp_ref(i,j+1))];
            disp(j);
            disp(size(tmp));
        end
        real_ref(i,:) = tmp;
    end
    ref_traj = real_ref;
    [row, col] = size(real_ref);
    disp(row);
    disp(col);
    num_steps = (num_steps - 1)*99 + 1;
end
disp(num_steps);
%t = 1:1:num_steps;

s_coeff = zeros(n,degree+1);
for i = 1:1:n
    s_coeff(i,:) = polyfit(t, ref_traj(i,:), degree);
end
% p1 = polyfit(t, ref_traj(1,:), degree);
% p2 = polyfit(t, ref_traj(2,:), degree);

% smooth the init ref_traj via polyfit
for i = 1:1:n
    ref_traj(i,:) = polyval(s_coeff(i,:), t);
end

%% setup and variables declaration %%
% if T <= 0
%     T = 1;
% end

if dt <= 0
    dt = 1e-4;
end

% if dt > T
%     error('dt should be smaller than or equal to T.');
% end

% num_steps = T/dt;
% result = main_2d_mobile_rrt();
% ref_traj = result.tree(:,path);
%[~, num_steps] = size(ref_traj);

if n <= 0
    n = 2;
end

if m <= 0
    m = 2;
end

if p <= 0
    p = 2;
end

[~, secureSensors] = size(pMinusS);
if secureSensors > p
    error('The number of secure sensors should be smaller than or equal to the total number of sensors.')
end

[rowA, colA] = size(A);
if rowA ~= n || colA ~= n
    error('A should be an n*n matrix.')
end

[rowB, colB] = size(B);
if rowB ~= n || colB ~= m
    error('B should be an n*m matrix.')
end

[rowC, colC] = size(C);
if rowC ~= p || colC ~= n
    error('C should be an p*n matrix.')
end

[rowQ, colQ] = size(Q);
if rowQ ~= n || colQ ~= n
    error('Q should be an n*n matrix.')
end

[rowR, colR] = size(R);
if rowR ~= m || colR ~= m
    error('R should be an m*m matrix.')
end

% A = eye(n); % 0
% B = eye(n);
% C = [1 1; 1 -1]; % eye(n)
% C_alpha = C(1,:);
% Q = eye(n);
% R = 1e-3*eye(n); % eye(n)
C_alpha = C(pMinusS-1,:);
Sigma_v_alpha = Sigma_v(pMinusS-1, pMinusS-1);
R_inv = inv(R);
P = zeros([n,n,num_steps]);

x = zeros(n, num_steps);
s = zeros(n, num_steps);
x_hat = zeros(n, num_steps);
x_alpha_hat = zeros(n,num_steps);
x_real = zeros(n,num_steps);
%x0 = zeros(n,1); % normrnd
x0 = start_point;
x(:,1) = x0;
x_hat(:,1) = x0;
x_alpha_hat(:,1) = x0;
x_real(:,1) = x0;

% Sigma_w = eye(n);
G = Sigma_w;
% Sigma_v = eye(n);
% Sigma_v_alpha = 1;
Sigma_v_inv = inv(Sigma_v);

Phi = zeros(n); % eye(n)
Theta = zeros(n, p); % Phi*C'*eye(n)
Phi_alpha = zeros(n);
Theta_alpha = zeros(n, secureSensors);

% ref_traj = linspace(0, 1, num_steps); % x = 21, y = [21 ... 16.5]
% ref_traj = [ref_traj; zeros(n-1, num_steps)];


%% reference trajetory %%


% t = log(sqrt(2)-1):1.7629e-04:log(sqrt(2)+1);
% x1 = 3*cosh(t) - 3*sqrt(2);
% x2 = 2.5*sinh(t) + 2.5;
% ref_traj = [x1; x2];
% p1 = polyfit(t, x1, degree); % degree = 5
% p2 = polyfit(t, x2, degree);

% t = 0:1e-4:1-1e-4;
% x1 = linspace(0,0,1e4);
% x2 = linspace(0,5,1e4);
% ref_traj = [x1; x2];
% p1 = polyfit(t,x1,3);
% p2 = polyfit(t,x2,3);


%% calculate P and s %%
for i = num_steps-1:-1:1
    P(:,:,i) = P(:,:,i+1) + dt*(A'*P(:,:,i+1) + P(:,:,i+1)*A - P(:,:,i+1)*B*R_inv*B'*P(:,:,i+1) + Q); % negative
    %s(:,i) = inv(eye(n) + dt*(-A' + P(:,:,i)*B*R_inv*B'))*(s(:,i+1) - dt*2*Q*ref_traj(:,i)); % ref_traj(:,i+1)
    %dsdt = (A' - B*R_inv*B'*P(:,:,i))'*s(:,i+1) - 2*Q*ref_traj(:,i+1);
    %dsdt = (A' - P(:,:,i)*B*R_inv*B')'*s(:,i+1) - 2*Q*ref_traj(:,i+1);
    dsdt = (A' - P(:,:,i)*B*R_inv*B')'*s(:,i+1) - Q*ref_traj(:,i+1);
    %dsdt = (A - B*R_inv*B'*P(:,:,i))'*s(:,i+1) - Q*ref_traj(:,i+1);
    s(:,i) = s(:,i+1) + dsdt*dt;
end


%% calculate K %%
sys = ss(A,[B G],C,0);
[~,K,~] = kalman(sys,Q,R,0);
% K

%% calculate Phi and Theta %%
Phi_alpha_prev = Phi_alpha + 0.0001;
Theta_alpha_prev = Theta_alpha + 0.0001;

%while abs(Phi - Phi_prev) < 0.0001 && abs(Theta_alpha - Theta_alpha_prev) < 0.0001
while abs(Phi_alpha - Phi_alpha_prev) >= 0.001 | abs(Theta_alpha - Theta_alpha_prev) >= 0.001
% for i = 1:2
    Phi_alpha_prev = Phi_alpha;
    Theta_alpha_prev = Theta_alpha;
    %dPhi_dt = A*Phi + Phi*A' + Sigma_w - Theta_alpha*C_alpha*inv(Sigma_v_alpha)*C_alpha'*Theta_alpha'; % C
    dPhi_alpha_dt = A*Phi_alpha + Phi_alpha*A' + Sigma_w - Phi_alpha*C_alpha'*inv(Sigma_v_alpha)*C_alpha*Phi_alpha';
    %dPhi_alpha_dt = A*Phi_alpha + Phi_alpha*A' + Sigma_w - Theta_alpha*C_alpha'*inv(Sigma_v_alpha)*C_alpha*Theta_alpha';
    Phi_alpha = Phi_alpha + dt*dPhi_alpha_dt;
    Theta_alpha = Phi_alpha*C_alpha'*inv(Sigma_v_alpha);
end

Phi_alpha
Theta_alpha
%% calculate gamma %%
%u_alpha = -0.5*R_inv*B'*P(:,:,1)*x_alpha_hat(:,1) - 0.5*R_inv*B'*s(:,1);

% gamma = InvokeSafetyBarrier(A,B,C_alpha,Sigma_w,Sigma_v,Sigma_v_alpha,R,K,Phi_alpha,Theta_alpha,n,m,p,s_coeff,degree,start_point,g_U,g_D);
gamma = 9.9216
%gamma = 10000;


%% reinitialize Phi%%
Phi = zeros(n);
Phi_alpha = zeros(n);


%% kalman filter and QCQP %%
for i = 2:num_steps
    % u
    %u_alpha = -0.5*R_inv*B'*P(:,:,i-1)*x_hat(:,i-1) - 0.5*R_inv*B'*s(:,i-1); % b(:,i-1)
    u_alpha = -0.5*R_inv*B'*P(:,:,i-1)*x_alpha_hat(:,i-1) - 0.5*R_inv*B'*s(:,i-1);
    
    opt_start = tic;
    options = optimoptions(@fmincon,'Display','off','Algorithm','interior-point',...
        'SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true,...
        'HessianFcn',@(z,lambda)quadhess(z,lambda,2*R,2*eye(n)));
    constraint_matrix{1} = eye(n);
    constraint_coefficient{1} = -2*u_alpha;
    constraint_constant{1} = u_alpha'*u_alpha-gamma^2;
    fun = @(z)quadobj(z,2*R,B'*(2*P(:,:,i-1)*x_hat(:,i-1) + 2*s(:,i)),0);
    nonlconstr = @(z)quadconstr(z,constraint_matrix,constraint_coefficient,constraint_constant);
    %x0 = [0;0]; % column vector
    x0 = start_point;
    [u_ast,fval,eflag,output,lambda] = fmincon(fun,x0,...
        [],[],[],[],[],[],nonlconstr,options);
    disp(u_ast)
    opt_time = toc(opt_start);
    % state space dynamics
    w = normrnd(0,1,n,1); % not exist
    dxdt = A*x(:,i-1) + B*u_ast + w;
    x(:,i) = x(:,i-1) + dxdt*dt;
    
    % observation
    v = normrnd(0,1,p,1); % randn(n,1)
    v_alpha = v(pMinusS,:);
    attack = rand(p,1);
    attack(pMinusS,1) = 0;
    y = C*x(:,i) + v + attack;
    y_alpha = C_alpha*x(:,i) + v_alpha;

    % Phi, Theta
    dPhi_alpha_dt = A*Phi_alpha + Phi_alpha*A' + Sigma_w - Phi_alpha*C_alpha'*inv(Sigma_v_alpha)*C_alpha*Phi_alpha';
    %dPhi_alpha_dt = A*Phi_alpha + Phi_alpha*A' + Sigma_w - Theta_alpha*C_alpha*Sigma_v_alpha*C_alpha'*Theta_alpha';
    %dPhi_dt = A*Phi + Phi*A' + Sigma_w - Theta*C*Sigma_v_inv*C'*Theta'; % C
    dPhi_dt = A*Phi + Phi*A' + Sigma_w - Phi*C'*Sigma_v_inv*C*Phi';
    Phi_alpha = Phi_alpha + dt*dPhi_alpha_dt;
    Theta_alpha = Phi_alpha*C_alpha'*inv(Sigma_v_alpha);
    Phi = Phi + dt*dPhi_dt;
    Theta = Phi*C'*Sigma_v_inv;

    % estimate
    dxhat_dt = A*x_hat(:,i-1) + B*u_ast + Theta*(y - C*x_hat(:,i-1));
    x_hat(:,i) = x_hat(:,i-1) + dt*dxhat_dt;

    dxhat_alpha_dt = A*x_alpha_hat(:,i-1) + B*u_ast + Theta_alpha*(y_alpha - C_alpha*x_alpha_hat(:,i-1));
    x_alpha_hat(:,i) = x_alpha_hat(:,i-1) + dt*dxhat_alpha_dt;

    %negative_dPdt = P*A + A'*P - P*B*R_inv*B'*P + Q; % repeat
    %P = P-negative_dPdt*dt;

    x_real(:,i) = x_real(:,i-1) + dt*(A*x_real(:,i-1) + B*u_ast);

    disp(i);
end


%% figure %%
t = 0:1:(num_steps-1);
for i = 1:1:n
    figure;
    plot(t, x_real(i,:));
    hold on;
    plot(t, ref_traj(i,:));
    xlabel('Time');
    ylabel('tracking/reference');
    legend('x', 'Reference');
end

figure;
plot(x_real(1,:), x_real(2,:), '-');
hold on;
plot(ref_traj(1,:), ref_traj(2,:), '--');
%plot(result.tree(1,path), result.tree(2,path), '--');
xlabel('Time');
ylabel('Trajectory');
legend('x', 'Reference');

% figure;
% plot(dt*(1:num_steps), x_real(1,:), '-');
% hold on;
% plot(dt*(1:num_steps), ref_traj(1,:), '--');
% xlabel('Time');
% ylabel('Trajectory');
% legend('x', 'Reference');

end
