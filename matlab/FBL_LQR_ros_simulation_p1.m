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
T = 25;
num_steps = 125;

start_point = [0; 0; 0; 0];
rd_tar = 0.05;
% rd_obs = 0.05;
target = [1; 0.25; 0; 0];
obs = -0.15;

%%
dt = T/num_steps;


%%
n = 4;

m = 2;

A = [0 0 1 0;
     0 0 0 1;
     0 0 0 0;
     0 0 0 0]; 
B = [0 0;
     0 0;
     1 0;
     0 1];
 
orientation = zeros(1,num_steps);
 
 
 
 
% with redundant x and y coordinates measurements
p = 6;
Sensor_index = 1:p;
pMinusS = [1,2,4,5];
F_r1 = [2];
F_r2 = [4];
F_r1_comp = Sensor_index(setxor(Sensor_index,F_r1));
F_r2_comp = Sensor_index(setxor(Sensor_index,F_r2));
F_r1r2_comp = Sensor_index(setxor(setxor(Sensor_index,F_r1),F_r2));
C = [1 0 0 0;
     1 0 0 0;
     0 1 0 0;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
Sigma_v = [1e-6 0 0 0 0 0; 
           0 1e-6 0 0 0 0; 
           0 0 1e-6 0 0 0;
           0 0 0 1e-6 0 0;
           0 0 0 0 1e-6 0;
           0 0 0 0 0 1e-6];
 
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


% s = zeros(n_l, num_steps);
% b = zeros(n_l,n_l,num_steps);
% s(:,num_steps)=[0;0;0;0];


%%
degree = 3;
% P = zeros([n,n,num_steps]);

%% initialization
% global dt


for i = 1:n-1
    eval(sprintf('syms x%d', i));
end

g_D = rd_tar^2;
% g_U = rd_obs^2;
% for i = 1:2
%     eval(sprintf('g_D = g_D - (x%d - target(i))^2', i));
%     eval(sprintf('g_U = g_U - (x%d - obs(i))^2', i));
% end












%% parameters initialization %%
C_r1 = C(F_r1_comp,:);
Sigma_v_r1 = Sigma_v(F_r1_comp, F_r1_comp);

C_r2 = C(F_r2_comp,:);
Sigma_v_r2 = Sigma_v(F_r2_comp, F_r2_comp);

C_r1r2 = C(F_r1r2_comp,:);
Sigma_v_r1r2 = Sigma_v(F_r1r2_comp, F_r1r2_comp);

R_inv = inv(R);
Sigma_v_inv = inv(Sigma_v);
Sigma_v_r1inv = inv(Sigma_v_r1);
Sigma_v_r2inv = inv(Sigma_v_r2);
Sigma_v_r1r2inv = inv(Sigma_v_r1r2);

x_hat = zeros(n, num_steps);
x_hat_r1 = zeros(n, num_steps);
x_hat_r2 = zeros(n, num_steps);
x_hat_r1r2 = zeros(n, num_steps);
x_real = zeros(n, num_steps);

x0 = start_point;
x_hat(:,1) = x0;
x_hat_r1(:,1) = x0;
x_hat_r2(:,1) = x0;
x_hat_r1r2(:,1) = x0;
x_real(:,1) = x0;

G = eye(n);


P = zeros([n,n,num_steps]);


u = zeros(m, num_steps);
u_ast = zeros(m, num_steps);

y = zeros(p, num_steps);
y_r1 = zeros(size(F_r1_comp,2), num_steps);
y_r2 = zeros(size(F_r2_comp,2), num_steps);
y_r1r2 = zeros(size(F_r1r2_comp,2), num_steps);

y(:,1) = C*x_hat(:,1);
y_r1(:,1) = y(F_r1_comp,1);
y_r2(:,1) = y(F_r2_comp,1);
y_r1r2(:,1) = y(F_r1r2_comp,1);

error = zeros(1,num_steps);
cost = zeros(1,num_steps);





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

Phi_r1 = zeros(n);
Theta_r1 = zeros(n,size(F_r1_comp,2));

Phi_r1_prev = Phi_r1 + 0.001;
Theta_r1_prev = Theta_r1 + 0.001;

Phi_r2 = zeros(n);
Theta_r2 = zeros(n,size(F_r2_comp,2));

Phi_r2_prev = Phi_r2 + 0.001;
Theta_r2_prev = Theta_r2 + 0.001;


while abs(Phi_r1 - Phi_r1_prev) >= 0.001
    Phi_r1_prev = Phi_r1;
    Theta_r1_prev = Theta_r1;
    
    dPhi_r1_dt = A*Phi_r1 + Phi_r1*A' + Sigma_w - Phi_r1*C_r1'*inv(Sigma_v_r1)*C_r1*Phi_r1';
    
    Phi_r1 = Phi_r1 + dt*dPhi_r1_dt;
    Theta_r1 = Phi_r1*C_r1'*inv(Sigma_v_r1);
    
    Phi_r2_prev = Phi_r2;
    Theta_r2_prev = Theta_r2;
    
    dPhi_r2_dt = A*Phi_r2 + Phi_r2*A' + Sigma_w - Phi_r2*C_r2'*inv(Sigma_v_r2)*C_r2*Phi_r2';
    
    Phi_r2 = Phi_r2 + dt*dPhi_r2_dt;
    Theta_r2 = Phi_r2*C_r2'*inv(Sigma_v_r2);
end


%% calculate gamma %%
% gm = InvokeSafetyBarrier(A,B,C_alpha,Sigma_w,Sigma_v,Sigma_v_alpha,R,K,Phi_alpha,Theta_alpha,P,n,m,p,s_coeff,degree,start_point,g_U,g_D);

gm = 10000;

threshold = 10000;
