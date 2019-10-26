%% initialization
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
%%% R              -- m*m cost matrixros
%%% Sigma_w        -- n*n autocorrelation matrix
%%% Sigma_v        -- p*p autocorrelation matrix
%%% Sigma_v_alpha  -- the covariance matrix of v_alpha
%%% ref_traj       -- n*num_steps polynomial reference trajectory
%%% degree         -- the degree of the polyfit for the reference trajectory

clear all
close all
clc

%% broken line
T = 100;
num_steps = 2000;
n = 3;
m = 2;
p = 3;
pMinusS = [1 2];
A = eye(3);
B = [1 0; 1 0; 0 1];
C = eye(3);
Sigma_w = [1e-6 0 0;
           0 1e-6 0;
           0 0 1e-6];
Sigma_v = [1e-6 0 0;
           0 1e-6 0;
           0 0 1e-6];
Q = [1 0 0; 0 1 0; 0 0 1];
R = [1 0; 0 1];
start_point = [0; 0; 0];
rd_tar = 1;
rd_obs = 1;
target = [2; 0.001; 0];
obs = [-1; 1];
%%
t = 0:0.05:100;

% kp1 = 1.1;
% kp2 = 1.1;
% kd1 = 0.9;
% kd2 = 0.9;
% t = log(sqrt(2)-1):1.7629e-02:log(sqrt(2)+1);

% x1 = sin(t/10);
% x2 = sin(t/20);

x1 = 0.01*t;
x2 = 0.01*t;

% x1 = 3*cosh(t)-3*sqrt(2);
% x2 = 2.5*sinh(t)+2.5;
% x2 = t.*(t<=1)+(-t+2).*(t>1);
%%
parametric_func = [x1; x2];
%%
dt = T/num_steps;
%%
s = zeros(2, num_steps);
b = zeros(2,2,num_steps);
s(:,num_steps)=[0;0];

n_l = 2;
m_l = 2;
p_l = 3;
A_l = eye(2);
B_l = dt*eye(2);
C_l = [1 0;
       0 1;
       0 1];
Q_l = eye(2);
start_point_l = [0; 0];
%%
degree = 3;
% P = zeros([n,n,num_steps]);

%% initialization
% global dt


for i = 1:n-1
    eval(sprintf('syms x%d', i));
end

g_D = rd_tar^2;
g_U = rd_obs^2;
for i = 1:n-1
    eval(sprintf('g_D = g_D - (x%d - target(i))^2', i));
    eval(sprintf('g_U = g_U - (x%d - obs(i))^2', i));
end

ref_traj = parametric_func;
diffrc = ref_traj(:,1);
ref_traj(:,:) = ref_traj(:,:) - diffrc;


%% s_coeff
s_coeff = zeros(n_l,degree+1);
for i = 1:1:n_l
    s_coeff(i,:) = polyfit(t, ref_traj(i,:), degree);
end

% smooth the init ref_traj via polyfit
for i = 1:1:n_l
    ref_traj(i,:) = polyval(s_coeff(i,:), t);
end


%% add the third dimension: angle
ref_length = size(ref_traj, 2);
ref_traj = [ref_traj; ones(1,ref_length)];
for i = 1:ref_length-1
    ref_traj(3,i) = atan((ref_traj(2,i+1)-ref_traj(2,i))/(ref_traj(1,i+1)-ref_traj(1,i)));
end
ref_traj(3,ref_length) = ref_traj(3,ref_length-1);

ref_traj_dot = zeros(3,ref_length);
for i = 2:ref_length
    ref_traj_dot(1,i) = (ref_traj(1,i)-ref_traj(1,i-1))/dt;
    ref_traj_dot(2,i) = (ref_traj(2,i)-ref_traj(2,i-1))/dt;
    ref_traj_dot(3,i) = (ref_traj(3,i)-ref_traj(3,i-1))/dt;
end

ref_traj_db_dot = zeros(3,ref_length);
for i = 1:ref_length-1
    ref_traj_db_dot(1,i) = (ref_traj_dot(1,i+1)-ref_traj_dot(1,i))/dt;
    ref_traj_db_dot(2,i) = (ref_traj_dot(2,i+1)-ref_traj_dot(2,i))/dt;
    ref_traj_db_dot(3,i) = (ref_traj_dot(3,i+1)-ref_traj_dot(3,i))/dt;
end

%% add the third dimension: xdd, ydd
%rd = [x_dot; y_dot];
ref_length = size(ref_traj, 2);
rd = zeros(2,ref_length-1);
for i = 1:ref_length-1
    rd(1,i) = ref_traj(1,i+1)-ref_traj(1,i);
    rd(2,i) = ref_traj(2,i+1)-ref_traj(2,i);
end
%rdd = drd/dt
rdd = zeros(2,ref_length-2);
for i = 1:ref_length-2
    rdd(1,i) = rd(1,i+1)-rd(1,i);
    rdd(2,i) = rd(2,i+1)-rd(2,i);
end

%% redefine start point and target
% start_point = ref_traj(:,1);
target = ref_traj(:, ref_length);


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



%% parameters initialization %%
C_alpha = C(pMinusS,:);
C_l_alpha = C_l(pMinusS,:);
Sigma_v_alpha = Sigma_v(pMinusS, pMinusS);
R_inv = inv(R);
Sigma_v_inv = inv(Sigma_v);



G = Sigma_w;

P = zeros([n,n,num_steps]);


Sigma_x = zeros(n, n, num_steps);
u = zeros(m, num_steps);
y = zeros(p, num_steps);

x_p = zeros(n,1);
Sigma_x_p = zeros(n,n);

error = zeros(1,num_steps);
cost = zeros(1,num_steps);
% Sigma_w = eye(n);



% Sigma_v = eye(n);
% Sigma_v_alpha = 1;
Sigma_w_l = [1e-6 0;
             0    1e-6];
Sigma_v_l = [1e-6 0    0;
             0    1e-6 0;
             0    0    1e-6];
Sigma_v_l_alpha = Sigma_v_l(pMinusS, pMinusS);
G = Sigma_w_l;


%% calculate K %%
sys = ss(A_l,[B_l G],C_l,0);
[~,K,~] = kalman(sys,Sigma_w_l,Sigma_v_l,0);


%% calculate Phi and Theta %%
Phi_l = zeros(n_l,n_l,num_steps);
Theta_l = zeros(n_l,p_l,num_steps);
Phi_l_alpha = zeros(n_l,n_l,num_steps);
Theta_l_alpha = zeros(n_l,secureSensors,num_steps);

for i = 2:num_steps
    dPhi_l_alpha_dt = A_l*Phi_l_alpha(:,:,i-1) + Phi_l_alpha(:,:,i-1)*A_l' + Sigma_w_l - Phi_l_alpha(:,:,i-1)*C_l_alpha'*inv(Sigma_v_l_alpha)*C_l_alpha*Phi_l_alpha(:,:,i-1)';
    Phi_l_alpha(:,:,i) = Phi_l_alpha(:,:,i-1) + dt*dPhi_l_alpha_dt;
    Theta_l_alpha(:,:,i) = Phi_l_alpha(:,:,i)*C_l_alpha'*inv(Sigma_v_l_alpha);

    dPhi_l_dt = A_l*Phi_l(:,:,i-1) + Phi_l(:,:,i-1)*A_l' + Sigma_w_l - Phi_l(:,:,i-1)*C_l'*inv(Sigma_v_l)*C_l*Phi_l(:,:,i-1)';
    Phi_l(:,:,i) = Phi_l(:,:,i-1) + dt*dPhi_l_dt;
    Theta_l(:,:,i) = Phi_l(:,:,i)*C_l'*inv(Sigma_v_l);
end




%% calculate gamma %%
% gamma = InvokeSafetyBarrier(A_l,B_l,C_l_alpha,Sigma_w_l,Sigma_v_l,Sigma_v_l_alpha,R,K,Phi_l_alpha(:,:,num_steps),Theta_l_alpha(:,:,num_steps),n_l,m_l,p_l,s_coeff,degree,start_point_l,g_U,g_D);
gamma = 10;
