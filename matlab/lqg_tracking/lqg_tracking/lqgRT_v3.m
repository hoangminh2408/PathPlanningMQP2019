function [] = lqgRT()

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

%% read configuration file
fileID = fopen('configuration.txt');
file = strings(21,1);
for i = 1:21
    file(i) = fgetl(fileID);
end
fclose(fileID);

if file(1) ~= ""
    T = eval(file(1));
    if T <= 0
        error('T must be a positive number.');
    end
else
    T = 1;
end

if file(2) ~= ""
    num_steps = eval(file(2));
    if mod(num_steps,1) ~= 0 || num_steps <= 0
        error('num_steps must be a positive integer.');
    end
else
    num_steps = 1e4;
end

if file(3) ~= ""
    n = eval(file(3));
    if mod(n,1) ~= 0 || n <= 0
        error('n must be a positive integer.');
    end
else
    n = 2;
end

if file(4) ~= ""
    m = eval(file(4));
    if mod(m,1) ~= 0 || m <= 0
        error('m must be a positive integer.');
    end
else
    m = 2;
end

if file(5) ~= ""
    p = eval(file(5));
    if mod(p,1) ~= 0 || p <= 0
        error('p must be a positive integer.');
    end
else
    p = 2;
end

if file(6) ~= ""
    pMinusS = eval(file(6));
    [pMinusSRow, secureSensors] = size(pMinusS);
    if pMinusSRow > 1
        error('pMinusS must be a row vector.');
    end
    
    if secureSensors > p
        error('The number of secure sensors should be smaller than or equal to the total number of sensors.');
    end
    
    isCorrect = true;
    for i = 1:secureSensors
        if mod(pMinusS(1,i),1) ~= 0 || pMinusS(1,i) <= 0 || pMinusS(1,i) > p
            isCorrect = false;
            break;
        end
    end
    
    if ~isCorrect
        error('pMinusS should be integer indices between 1 and %d.\n', p);
    end
else
    pMinusS = 2;
end

if file(7) ~= ""
    A = eval(file(7));
    if size(A,1) ~= n || size(A,2) ~= n
        error('A should be an n by n matrix.');
    end
else
    A = eye(n);
end

if file(8) ~= ""
    B = eval(file(8));
    if size(B,1) ~= n || size(B,2) ~= m
        error('B should be an n by m matrix.');
    end
else
    B = eye(n,m);
end

if file(9) ~= ""
    C = eval(file(9));
    if size(C,1) ~= p || size(C,2) ~= n
        error('C should be a p by n matrix.');
    end
else
    C = eye(p,n);
end

if file(10) ~= ""
    Sigma_w = eval(file(10));
    if size(Sigma_w,1) ~= n || size(Sigma_w,2) ~= n
        error('Sigma_w should be an n by n matrix.');
    end
else
    Sigma_w = eye(n);
end

if file(11) ~= ""
    Sigma_v = eval(file(11));
    if size(Sigma_v,1) ~= p || size(Sigma_v,2) ~= p
        error('Sigma_v should be a p by p matrix.');
    end
else
    Sigma_v = eye(p);
end

if file(12) ~= ""
    Q = eval(file(12));
    if size(Q,1) ~= n || size(Q,2) ~= n
        error('Q should be an n by n matrix.');
    end
else
    Q = eye(n);
end

if file(13) ~= ""
    R = eval(file(13));
    if size(R,1) ~= m || size(R,2) ~= m
        error('R should be an m by m matrix.');
    end
else
    R = eye(m);
end

if file(14) ~= ""
    start_point = eval(file(14));
    if size(start_point,1) ~= n || size(start_point,2) ~= 1
        error('start_point should be an n by 1 vector.');
    end
else
    start_point = [0; 0];
end

if file(15) ~= ""
    rd_tar = eval(file(15));
    if size(rd_tar,1) > 1 || size(rd_tar,2) > 1
        error('rd_tar should be a number.')
    end
    
    if rd_tar < 0
        error('rd_tar should be a nonnegative value.');
    end
else
    rd_tar = 1;
end

if file(16) ~= ""
    rd_obs = eval(file(16));
    if size(rd_tar,1) > 1 || size(rd_tar,2) > 1
        error('rd_obs should be a number.')
    end
    
    if rd_tar < 0
        error('rd_obs should be a nonnegative value.');
    end
else
    rd_obs = 1;
end

if file(17) ~= ""
    target = eval(file(17));
    if size(target,1) ~= n || size(target,2) ~= 1
        error('target should be an n by 1 vector.');
    end
else
    target = [0; 5];
end

if file(18) ~= ""
    obs = eval(file(18));
    if size(obs,1) ~= n || size(obs,2) ~= 1
        error('obs should be an n by 1 vector.');
    end
    
    if norm(target - obs) <= (rd_tar + rd_obs)
        error('Target and obstacle areas overlap. Please enter a further obstacle.');
    end
else
    obs = [0; 2.5];
end

if file(19) ~= ""
    t = eval(file(19));
    if size(t, 2) ~= num_steps
        error('Please make sure that the number of t is equal to the number of iterations during T.');
    end
else
    t = 0:(-4/(1e4-1)):-4;
end

if file(20) ~= ""
    parametric_func = eval(file(20));
    if size(parametric_func, 1) ~= n || size(parametric_func, 2) ~= num_steps
        error('parametric_func should be an n by 1 vector (using t as the parameter).');
    end
else
    parametric_func = zeros(n,1);
end

if file(21) ~= ""
    degree = eval(file(21));
    if mod(degree,1) ~= 0 || degree <= 0
        error('degree should be a positive integer.');
    end
else
    degree = 5;
end


%% system setup %%
dt = T/num_steps;


%% start point, target, and obstacle %%
for i = 1:n
    eval(sprintf('syms x%d', i));
end

g_D = rd_tar^2;
g_U = rd_obs^2;
for i = 1:n
    eval(sprintf('g_D = g_D - (x%d - target(1))^2', i));
    eval(sprintf('g_U = g_U - (x%d - obs(1))^2', i));
end

%% parametric functions, reference trajectory %%
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
C_alpha = C(pMinusS,:);
Sigma_v_alpha = Sigma_v(pMinusS, pMinusS);
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
Phi_alpha_prev = Phi_alpha + 0.001;
Theta_alpha_prev = Theta_alpha + 0.001;

%while abs(Phi - Phi_prev) < 0.0001 && abs(Theta_alpha - Theta_alpha_prev) < 0.0001
while abs(Phi_alpha - Phi_alpha_prev) >= 0.001 | abs(Theta_alpha - Theta_alpha_prev) >= 0.001
    Phi_alpha_prev = Phi_alpha;
    Theta_alpha_prev = Theta_alpha;
    %dPhi_dt = A*Phi + Phi*A' + Sigma_w - Theta_alpha*C_alpha*inv(Sigma_v_alpha)*C_alpha'*Theta_alpha'; % C
    dPhi_alpha_dt = A*Phi_alpha + Phi_alpha*A' + Sigma_w - Phi_alpha*C_alpha'*inv(Sigma_v_alpha)*C_alpha*Phi_alpha';
    %dPhi_alpha_dt = A*Phi_alpha + Phi_alpha*A' + Sigma_w - Theta_alpha*C_alpha'*inv(Sigma_v_alpha)*C_alpha*Theta_alpha';
    Phi_alpha = Phi_alpha + dt*dPhi_alpha_dt;
    Theta_alpha = Phi_alpha*C_alpha'*inv(Sigma_v_alpha);
end


%% calculate gamma %%
%u_alpha = -0.5*R_inv*B'*P(:,:,1)*x_alpha_hat(:,1) - 0.5*R_inv*B'*s(:,1);

gm = InvokeSafetyBarrier(A,B,C_alpha,Sigma_w,Sigma_v,Sigma_v_alpha,R,K,Phi_alpha,Theta_alpha,n,m,p,s_coeff,degree,start_point,g_U,g_D);

% gamma = 10000;


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
    constraint_constant{1} = u_alpha'*u_alpha-gm^2;
    fun = @(z)quadobj(z,2*R,B'*(2*P(:,:,i-1)*x_hat(:,i-1) + 2*s(:,i)),0);
    nonlconstr = @(z)quadconstr(z,constraint_matrix,constraint_coefficient,constraint_constant);
    %x0 = [0;0]; % column vector
    x0 = start_point;
    [u_ast,fval,eflag,output,lambda] = fmincon(fun,x0,...
        [],[],[],[],[],[],nonlconstr,options);
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

disp(gm);

% figure;
% plot(dt*(1:num_steps), x_real(1,:), '-');
% hold on;
% plot(dt*(1:num_steps), ref_traj(1,:), '--');
% xlabel('Time');
% ylabel('Trajectory');
% legend('x', 'Reference');

end