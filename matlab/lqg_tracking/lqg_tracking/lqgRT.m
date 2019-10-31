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


%% system setup %%
while 1
    T = input('Final time of the system: ');
    if T <= 0
        disp('Please enter a positive number.');
        continue;
    end
    break;
end
   
while 1
    num_steps = input('Number of iterations during T: ');
    if mod(num_steps,1) ~= 0 || num_steps <= 0
        disp('Please enter a positive integer.');
        continue;
    end
    break;
end
dt = T/num_steps;

while 1
    n = input('Dimension of the system state: ');
    if mod(n,1) ~= 0 || n <= 0
        disp('Please enter a positive integer.');
        continue;
    end
    break;
end

while 1
    m = input('Dimension of the system input: ');
    if mod(m,1) ~= 0 || m <= 0
        disp('Please enter a positive integer.');
        continue;
    end
    break;
end

while 1
    p = input('Dimension of the system observation: ');
    if mod(p,1) ~= 0 || p <= 0
        disp('Please enter a positive integer.');
        continue;
    end
    break;
end

while 1
    pMinusS = input('Index list of the secure sensors(row vector): ');
    [pMinusSRow, secureSensors] = size(pMinusS);
    if pMinusSRow > 1
        disp('Please enter a row vector.');
        continue;
    end
    
    if secureSensors > p
        disp('The number of secure sensors should be smaller than or equal to the total number of sensors.');
        continue;
    end
    
    for i = 1:secureSensors
        isCorrect = true;
        if mod(pMinusS(1,i),1) ~= 0 || pMinusS(1,i) <= 0 || pMinusS(1,i) > p
            isCorrect = false;
            fprintf('Please enter integer index between 1 and %d.\n', p);
            break;
        end
    end
    
    if ~isCorrect
        continue;
    end
    
    break;
end

while 1
    A = input('nxn state matrix: ');
    if size(A,1) ~= n || size(A,2) ~= n
        disp('Please enter an n by n matrix.');
        continue;
    end
    break;
end

while 1
    B = input('nxm input matrix: ');
    if size(B,1) ~= n || size(B,2) ~= m
        disp('Please enter an n by m matrix.');
        continue;
    end
    break;
end

while 1
    C = input('pxn output matrix: ');
    if size(C,1) ~= p || size(C,2) ~= n
        disp('Please enter a p by n matrix.');
        continue;
    end
    break;
end

while 1
    Sigma_w = input('nxn autocorrelation matrix: ');
    if size(Sigma_w,1) ~= n || size(Sigma_w,2) ~= n
        disp('Please enter an n by n matrix.');
        continue;
    end
    break;
end

while 1
    Sigma_v = input('pxp autocorrelation matrix: ');
    if size(Sigma_v,1) ~= p || size(Sigma_v,2) ~= p
        disp('Please enter a p by p matrix.');
        continue;
    end
    break;
end

while 1
    Q = input('nxn cost matrix: ');
    if size(Q,1) ~= n || size(Q,2) ~= n
        disp('Please enter an n by n matrix.');
        continue;
    end
    break;
end

while 1
    R = input('mxm cost matrix: ');
    if size(R,1) ~= m || size(R,2) ~= m
        disp('Please enter an m by m matrix.');
        continue;
    end
    break;
end


%% start point, target, and obstacle %%
while 1
    start_point = input('Start point: ');
    if size(start_point,1) ~= n || size(start_point,2) ~= 1
        disp('Please enter an n by 1 vector.');
        continue;
    end
    break;
end

while 1
    rd_tar = input('Radius of the target area: ');
    if size(rd_tar,1) > 1 || size(rd_tar,2) > 1
        disp('Please enter a number.')
        continue;
    end
    
    if rd_tar < 0
        disp('Please enter a nonnegative value.');
        continue;
    end
    break;
end

while 1
    rd_obs = input('Radius of the obstacle area: ');
    if size(rd_obs,1) > 1 || size(rd_obs,2) > 1
        disp('Please enter a number.')
        continue;
    end
    
    if rd_obs < 0
        disp('Please enter a nonnegative value.');
        continue;
    end
    break;
end

while 1
    target = input('Center of the target area: ');
    if size(target,1) ~= n || size(target,2) ~= 1
        disp('Please enter an n by 1 vector.');
        continue;
    end
    break;
end

while 1
    obs = input('Center of the obstacle area: ');
    if size(obs,1) ~= n || size(obs,2) ~= 1
        disp('Please enter an n by 1 vector.');
        continue;
    end
    
    if norm(target - obs) <= (rd_tar + rd_obs)
        disp('Target and obstacle areas overlap. Please enter a further obstacle.');
        continue;
    end
    
    break;
end

for i = 1:n
    eval(sprintf('syms x%d', i));
end

g_D = rd_tar^2;
g_U = rd_obs^2;
for i = 1:n
    eval(sprintf('g_D = g_D - (x%d - target(i))^2', i));
    eval(sprintf('g_U = g_U - (x%d - obs(i))^2', i));
end


%% parametric functions, reference trajectory %%
while 1
    t = input('Domain of t(in terms of start:step:end): ');
    if size(t, 2) ~= num_steps
        disp('Please make sure that the number of t is equal to the number of iterations during T');
        continue;
    end
    break;
end

pf = zeros(n,num_steps);
for i = 1:n
    fprintf('%dth Parametric function(using t as the parameter):', i);
    pf(i,:) = input(' ');
end
ref_traj = pf;

while 1
    degree = input('Degree of the polyfit for the reference trajectory: ');
    if mod(degree,1) ~= 0 || degree <= 0
        disp('Please enter a positive integer.');
        continue;
    end
    break;
end


%% reference trajectory enlargement %%
if num_steps <= 1000
    disp(num_steps);
    temp_ref = ref_traj;
    real_ref = zeros(n,((num_steps - 1)*99 + 1));
    disp(size(real_ref));
    
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

s_coeff = zeros(n,degree+1);
for i = 1:1:n
    s_coeff(i,:) = polyfit(t, ref_traj(i,:), degree);
end

% smooth the init ref_traj via polyfit
for i = 1:1:n
    ref_traj(i,:) = polyval(s_coeff(i,:), t);
end


%% parameters initialization %%
C_alpha = C(pMinusS,:);
Sigma_v_alpha = Sigma_v(pMinusS, pMinusS);
R_inv = inv(R);
Sigma_v_inv = inv(Sigma_v);

x = zeros(n, num_steps);
x_hat = zeros(n, num_steps);
x_alpha_hat = zeros(n,num_steps);
x_real = zeros(n,num_steps);
x0 = start_point;
x(:,1) = x0;
x_hat(:,1) = x0;
x_alpha_hat(:,1) = x0;
x_real(:,1) = x0;

G = Sigma_w;

Phi_alpha = zeros(n);
Theta_alpha = zeros(n, secureSensors);
P = zeros([n,n,num_steps]);
s = zeros(n, num_steps);


%% calculate P and s %%
for i = num_steps-1:-1:1
    P(:,:,i) = P(:,:,i+1) + dt*(A'*P(:,:,i+1) + P(:,:,i+1)*A - P(:,:,i+1)*B*R_inv*B'*P(:,:,i+1) + Q);    
    dsdt = (A' - P(:,:,i)*B*R_inv*B')'*s(:,i+1) - Q*ref_traj(:,i+1);
    s(:,i) = s(:,i+1) + dsdt*dt;
end


%% calculate K %%
sys = ss(A,[B G ],C,0);
[~,K,~] = kalman(sys,Q,R,0);


%% calculate Phi and Theta %%
Phi_alpha_prev = Phi_alpha + 0.001;
Theta_alpha_prev = Theta_alpha + 0.001;

while abs(Phi_alpha - Phi_alpha_prev) >= 0.001 | abs(Theta_alpha - Theta_alpha_prev) >= 0.001
    Phi_alpha_prev = Phi_alpha;
    Theta_alpha_prev = Theta_alpha;
    dPhi_alpha_dt = A*Phi_alpha + Phi_alpha*A' + Sigma_w - Phi_alpha*C_alpha'*inv(Sigma_v_alpha)*C_alpha*Phi_alpha';
    Phi_alpha = Phi_alpha + dt*dPhi_alpha_dt;
    Theta_alpha = Phi_alpha*C_alpha'*inv(Sigma_v_alpha);
end


%% calculate gamma %%
gamma = InvokeSafetyBarrier(A,B,C_alpha,Sigma_w,Sigma_v,Sigma_v_alpha,R,K,Phi_alpha,Theta_alpha,n,m,p,s_coeff,degree,start_point,g_U,g_D);


%% reinitialize Phi %%
Phi = zeros(n); 
Phi_alpha = zeros(n);


%% kalman filter and QCQP %%
for i = 2:num_steps
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

    x0 = start_point;
    [u_ast,fval,eflag,output,lambda] = fmincon(fun,x0,...
        [],[],[],[],[],[],nonlconstr,options);
    opt_time = toc(opt_start);
    
    % state space dynamics
    w = normrnd(0,1,n,1); 
    dxdt = A*x(:,i-1) + B*u_ast + w; 
    x(:,i) = x(:,i-1) + dxdt*dt;
    
    % observation
    v = normrnd(0,1,p,1); 
    v_alpha = v(pMinusS,:);
    attack = rand(p,1);
    attack(pMinusS,1) = 0;
    y = C*x(:,i) + v + attack;
    y_alpha = C_alpha*x(:,i) + v_alpha;
    
    % Phi, Theta
    dPhi_alpha_dt = A*Phi_alpha + Phi_alpha*A' + Sigma_w - Phi_alpha*C_alpha'*inv(Sigma_v_alpha)*C_alpha*Phi_alpha';
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

end