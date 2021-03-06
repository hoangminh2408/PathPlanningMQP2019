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
T = 40;
num_steps =200;
% n = 3;
% m = 2;
% p = 3;

% A = eye(3);
% B = [1 0; 1 0; 0 1];
% C = eye(3);
% Sigma_w = [1e-6 0 0; 
%            0 1e-6 0; 
%            0 0 1e-6];
% Sigma_v = [1e-6 0 0; 
%            0 1e-6 0; 
%            0 0 1e-6];
% Q = [1 0 0; 0 1 0; 0 0 1];
% R = [1 0; 0 1];
start_point = [0; 0; 0];
rd_tar = 1;
rd_obs = 1;
target = [2; 0.001; 0];
obs = [-1; 1];
%%
% 8
% t = 0:0.1:100;
% x1 = sin(t/10);
% x2 = sin(t/20);

% kp1 = 1.1;
% kp2 = 1.1;
% kd1 = 0.9;
% kd2 = 0.9;

% hyperbola
t = linspace(1,100,num_steps);
x1 = 0.8*sin(t/10);
x2 = 0.8*sin(t/20);
% x1 = 3*cosh(t)-3*sqrt(2);
% x2 = 2.5*sinh(t)+2.5;

% x1 = 0.5*sinh(t)+0.5;
% x2 = 0.5*sqrt(2)-0.5*cosh(t);
% x1 = 0.01*t;
% x2 = 0.01*t;


% x2 = t.*(t<=1)+(-t+2).*(t>1);
%%
parametric_func = [x1; x2];
%%
dt = T/num_steps;


%%
n = 4;
n_l = 4;
m = 2;

A = [0 0 1 0;
     0 0 0 1;
     0 0 0 0;
     0 0 0 0]; 
B = [0 0;
     0 0;
     1 0;
     0 1];
 

 
 
 
 
 
 % with redundant y coordinate measurement
p = 5;
pMinusS = [1,2,4,5];
C = [1 0 0 0;
     0 1 0 0;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
Sigma_v = [1e-6 0 0 0 0; 
           0 1e-6 0 0 0; 
           0 0 6*1e-4 0 0;
           0 0 0 1e-6 0;
           0 0 0 0 1e-6];
 
% without redundant y coordinate measurement
% p = 4;
% pMinusS = [1,3,4];
% C = [1 0 0 0;
%      0 1 0 0;
%      0 0 1 0;
%      0 0 0 1];
% Sigma_v = [1e-6 0 0 0; 
%            0 1e-6 0 0; 
%            0 0 1e-6 0;
%            0 0 0 1e-6];


Q = eye(4);
R = eye(2);
Sigma_w = [1e-6 0 0 0; 
           0 1e-6 0 0; 
           0 0 1e-6 0;
           0 0 0 1e-6];


s = zeros(n_l, num_steps);
b = zeros(n_l,n_l,num_steps);
s(:,num_steps)=[0;0;0;0];


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
for i = 1:2
    eval(sprintf('g_D = g_D - (x%d - target(i))^2', i));
    eval(sprintf('g_U = g_U - (x%d - obs(i))^2', i));
end

ref_traj = parametric_func;






%% add the third dimension: angle
ref_length = size(ref_traj, 2);
ref_traj = [ref_traj; ones(1,ref_length)];
for i = 1:ref_length-1
    ref_traj(3,i) = atan((ref_traj(2,i+1)-ref_traj(2,i))/(ref_traj(1,i+1)-ref_traj(1,i)));
end
ref_traj(3,ref_length) = ref_traj(3,ref_length-1);
start_point = ref_traj(:,1);

ref_traj_dot = zeros(3,ref_length);
for i = 2:ref_length
    ref_traj_dot(1,i) = (ref_traj(1,i)-ref_traj(1,i-1))/dt;
    ref_traj_dot(2,i) = (ref_traj(2,i)-ref_traj(2,i-1))/dt;
    ref_traj_dot(3,i) = (ref_traj(3,i)-ref_traj(3,i-1))/dt;
end
start_point = [start_point; ref_traj_dot(1:2, 1)];

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


ref_traj = [ref_traj(1:2,:); ref_traj_dot(1:2,:)];


s_coeff = zeros(n,degree+1);
for i = 1:1:n
    s_coeff(i,:) = polyfit(t, ref_traj(i,:), degree);
end
% p1 = polyfit(t, ref_traj(1,:), degree); 
% p2 = polyfit(t, ref_traj(2,:), degree);

% smooth the init ref_traj via polyfit
for i = 1:1:n
%     ref_traj(i,:) = polyval(s_coeff(i,:), t);
end


diffrc = ref_traj(:,1);
ref_traj(:,:) = ref_traj(:,:) - diffrc;
start_point = ref_traj(:,1);

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
Sigma_v_alpha = Sigma_v(pMinusS, pMinusS);
R_inv = inv(R);
Sigma_v_inv = inv(Sigma_v);

x_hat = zeros(n, num_steps);
x_alpha_hat = zeros(n, num_steps);
x_real = zeros(n,num_steps);
x0 = ref_traj(:,1);
x_hat(:,1) = x0;
x_alpha_hat(:,1) = x0;
x_real(:,1) = x0;

% x0_l = [start_point(1:2,1); start_point(4:5,1)];
% x_hat_l = zeros(n_l, num_steps);
% x_hat_l(:,1) = x0_l;

G = eye(n);

P = zeros([n,n,num_steps]);


Sigma_x = zeros(n, n, num_steps);
u = zeros(m, num_steps);
u_ast = zeros(m, num_steps);
u_diff = zeros(m, num_steps);
y = zeros(p, num_steps);
y_alpha = zeros(secureSensors, num_steps);
y_dist = zeros(1, num_steps);

x_p = zeros(n,1);
Sigma_x_p = zeros(n,n);

error = zeros(1,num_steps);
cost = zeros(1,num_steps);
% Sigma_w = eye(n);



% Sigma_v = eye(n);
% Sigma_v_alpha = 1;




%% calculate P and s %%
P = zeros([n,n]);
P_prev = P+0.001;

while abs(P - P_prev) >= 0.001
    P_prev = P;
    P = P + (2*Q + A'*P + P*A - 0.5*P*B*inv(R)*B'*P)*dt;
    
end


%% calculate K %%
sys = ss(A,[B G],C,0);
[~,K,~] = kalman(sys,Sigma_w,Sigma_v,0);


%% calculate Phi and Theta %%
% Phi = zeros(n,n,num_steps); 
Phi_alpha = zeros(n);
% Theta = zeros(n,p,num_steps);
Theta_alpha = zeros(n,secureSensors);

Phi_alpha_prev = Phi_alpha + 0.001;
Theta_alpha_prev = Theta_alpha + 0.001;
% ~all(all(abs(Phi_alpha - Phi_alpha_prev) <= 0.001) <= 0.001)
% ~all(all(abs(Theta_alpha - Theta_alpha_prev) <= 0.001) <= 0.001)

% while abs(Phi_alpha - Phi_alpha_prev) >= 0.001 | abs(Theta_alpha - Theta_alpha_prev) >= 0.001
while abs(Phi_alpha - Phi_alpha_prev) >= 0.001
    Phi_alpha_prev = Phi_alpha;
    Theta_alpha_prev = Theta_alpha;
    %dPhi_dt = A*Phi + Phi*A' + Sigma_w - Theta_alpha*C_alpha*inv(Sigma_v_alpha)*C_alpha'*Theta_alpha'; % C
    dPhi_alpha_dt = A*Phi_alpha + Phi_alpha*A' + Sigma_w - Phi_alpha*C_alpha'*inv(Sigma_v_alpha)*C_alpha*Phi_alpha';
    %dPhi_alpha_dt = A*Phi_alpha + Phi_alpha*A' + Sigma_w - Theta_alpha*C_alpha'*inv(Sigma_v_alpha)*C_alpha*Theta_alpha';
    Phi_alpha = Phi_alpha + dt*dPhi_alpha_dt;
    Theta_alpha = Phi_alpha*C_alpha'*inv(Sigma_v_alpha);
end
%% calculate gamma %%
% gm = InvokeSafetyBarrier(A,B,C_alpha,Sigma_w,Sigma_v,Sigma_v_alpha,R,K,Phi_alpha,Theta_alpha,P,n,m,p,s_coeff,degree,start_point,g_U,g_D);

gm = 10000;


