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
num_steps = 1e2;
n = 3;
m = 2;
p = 3;
pMinusS = [2];
A = eye(3);
B = [1 0; 1 0; 0 1];
C = eye(3);
Sigma_w = eye(3);
Sigma_v = eye(3);
Q = [1 0 0; 0 1 0; 0 0 1];
R = [1 0; 0 1];
start_point = [0; 0; 0];
rd_tar = 1;
rd_obs = 1;
target = [2; 0.001; 0];
obs = [-1; 1];
%%
t = 0:99;

kp1 = 1;
kp2 = 1;
kd1 = 0.8;
kd2 = 0.8;
% t = log(sqrt(2)-1):1.7629e-02:log(sqrt(2)+1);

x1 = sin(t/10);
x2 = sin(t/20);

% x1 = 0.01*t;
% x2 = 0.01*t;

% x1 = 3*cosh(t)-3*sqrt(2);
% x2 = 2.5*sinh(t)+2.5;
% x2 = t.*(t<=1)+(-t+2).*(t>1);
%%
parametric_func = [x1; x2];
%%
s = zeros(n, num_steps);
b = zeros(n,n,num_steps);
s(:,num_steps)=[0;0;0];
b(:,:,num_steps)=zeros(3);
%%
degree = 3;
% P = zeros([n,n,num_steps]);

%% initialization
% global dt
dt = T/num_steps;

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


%% add the third dimension: angle
ref_length = size(ref_traj, 2);
ref_traj = [ref_traj; ones(1,ref_length)];
for i = 1:ref_length-1
    ref_traj(3,i) = atan((ref_traj(2,i+1)-ref_traj(2,i))/(ref_traj(1,i+1)-ref_traj(1,i)));
end
ref_traj(3,ref_length) = ref_traj(3,ref_length-1);

ref_traj_dot = zeros(3,ref_length);
for i = 2:ref_length
    ref_traj_dot(1,i) = ref_traj(1,i)-ref_traj(1,i-1);
    ref_traj_dot(2,i) = ref_traj(2,i)-ref_traj(2,i-1);
    ref_traj_dot(3,i) = ref_traj(3,i)-ref_traj(3,i-1);
end

ref_traj_db_dot = zeros(3,ref_length);
for i = 1:ref_length-1
    ref_traj_db_dot(1,i) = ref_traj_dot(1,i+1)-ref_traj_dot(1,i);
    ref_traj_db_dot(2,i) = ref_traj_dot(2,i+1)-ref_traj_dot(2,i);
    ref_traj_db_dot(3,i) = ref_traj_dot(3,i+1)-ref_traj_dot(3,i);
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
Sigma_v_alpha = Sigma_v(pMinusS, pMinusS);
R_inv = inv(R);
Sigma_v_inv = inv(Sigma_v);

x_hat = zeros(n, num_steps);
x_alpha_hat = zeros(n,num_steps);
x_real = zeros(n,num_steps);
x0 = start_point;
x_hat(:,1) = x0;
x_alpha_hat(:,1) = x0;
x_real(:,1) = x0;

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



%% turtlebot
% rosinit;
robot = rospublisher('/cmd_vel');
laser = rossubscriber('/scan');
imu = rossubscriber('/imu');
odom = rossubscriber('/odom');
stmsg = rosmessage(odom);
msg = rosmessage(robot);

%% State receive

% state.Pose.Pose.Orientation


%% Loop
finish=false;
set(gcf,'CurrentCharacter','@'); % set to a dummy character
figure(1)
plot(ref_traj(1,:),ref_traj(2,:))
hold on
% figure(2)
% hold on
% figure(3)
% hold on
% figure(4)
% hold on
% figure(5)
% hold on

first_step_angle = atan((ref_traj(2,2) - ref_traj(2,1))/(ref_traj(1,2) - ref_traj(1,1)));
init_angle = 0;
theta = first_step_angle-init_angle(1);
% msg.Angular.Z = 0.35;
% send(robot,msg);
% while abs(theta) > 0.08 && abs(6.28-abs(theta)) >0.08
%     state = receive(odom,3);
%     quat = state.Pose.Pose.Orientation;
%     angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
%     theta = first_step_angle-angles(1)
%     if strcmpi(get(gcf,'CurrentCharacter'),'e')
%         break;
%     end
% end
% msg.Angular.Z = 0;
% send(robot,msg);
state_init = [0;0;1e-4];
%Initial B
B_ind = 0;
% B = 0.01.*[B_can(:,2*(18+B_ind)+1),B_can(:,2*(18+B_ind)+2)];
B = [cos(0.0079),0;
         sin(0.0079),0;
         0         ,1];

y(:,1) = state_init;
plot(y(1,1),y(2,1))
hold on
% while state(3,1) > pi | state(3,1) < -pi
%     state(3,2) = state(3,1) - 2*pi*sign(state(3,1));
% end

% move_s(u(1,1),u(2,1));
% B_can = [];
% for i=-175:10:185
%      B = [cos(deg2rad(i)),0;
%          sin(deg2rad(i)),0;
%          0              ,1];
%      B_can = [B_can,B];
% end
%% calculate P and s %%
% for j = 1:37
%     B_temp = [B_can(:,2*j-1),B_can(:,2*j)];

P = b-Q;
% end

% for i = num_steps-1:-1:1
%     P(:,:,i,j) = P(:,:,i+1,j) + dt*(A'*P(:,:,i+1,j) + P(:,:,i+1,j)*A - P(:,:,i+1,j)*B_temp*R_inv*B_temp'*P(:,:,i+1,j) + Q);
%     dsdt = (A' - P(:,:,i,j)*B_temp*R_inv*B_temp')'*s(:,i+1,j) - Q*ref_traj(:,i+1);
%     s(:,i,j) = s(:,i+1,j) + dsdt*dt;
% end
% for j = num_steps-1:-1:1
%     P(:,:,j) = A'*(P(:,:,j)-P(:,:,j+1)*B*inv(B'*P(:,:,j+1)*B+R)*B'*P(:,:,j+1))*A+Q+(P(:,:,j)-Q)*ref_traj(:,j);
% end

Phi = zeros(n,n,num_steps);
% Phi_alpha = zeros(n);
Theta = zeros(n,p,num_steps);
Theta(:,:,1) = Phi(:,:,1)*C'*Sigma_v_inv;

for i = 2:num_steps
%     dPhi_dt = A*Phi(:,:,i-1) + Phi(:,:,i-1)*A' + Sigma_w - Phi(:,:,i-1)*C'*Sigma_v_inv*C*Phi(:,:,i-1)';
%     Phi(:,:,i) = Phi(:,:,i-1) + dt*dPhi_dt;
%     Theta(:,:,i) = Phi(:,:,i)*C'*Sigma_v_inv;

    Phi_temp = A*Phi(:,:,i-1)*A'+Sigma_w;
    Theta(:,:,i) = Phi_temp*C'*inv(C*Phi_temp*C'+Sigma_v);
    Phi(:,:,i) = A*Phi_temp*A' + Sigma_w - Phi_temp*C'*inv(C*Phi_temp*C'+Sigma_v)*C*Phi_temp';
end

u(1,1) = ref_traj_db_dot(1,1) + kp1*(ref_traj(1,1)-y(1,1));
u(2,1) = ref_traj_db_dot(2,1) + kp2*(ref_traj(2,1)-y(2,1));
Xi = u(1,1)*cos(y(3,1))*dt+u(2,1)*sin(y(3,1))*dt
omega = (u(2,1)*cos(y(3,1))-u(1,1)*sin(y(3,1)))/Xi
if omega > 0.3
    omega = 0.3;
elseif omega < -0.3
       omega = -0.3
end
msg.Angular.Z = omega;
msg.Linear.X = Xi;
send(robot,msg);
tic;
while toc<1
    plot(y(1,i),y(2,i),'.')
    hold on
    if strcmpi(get(gcf,'CurrentCharacter'),'e')
        rosshutdown;
        break;
    end
end


for i = 2:num_steps
    disp(i)
    tic;

    state = receive(odom,3);

    tbot_x = state.Pose.Pose.Position.X;
    tbot_y = state.Pose.Pose.Position.Y;

    quat = state.Pose.Pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
%     y(:,i) = [tbot_x+rand; tbot_y];
    y(:,i) = [tbot_x; tbot_y; angles(1)];

    x_temp = A*x_hat(:,i-1) + B*u(:,i-1);
    x_hat(:,i) = x_temp + Theta(:,:,i)*(y(:,i)-C*x_temp);

    %Initial B
%     B_ind = round(rad2deg(state(3,i-1))/10);

%     B =0.01.* [B_can(:,2*(18+B_ind)+1),B_can(:,2*(18+B_ind)+2)];

    %%
%     slope_original = (ref_traj(2,i) - ref_traj(2,i-1))/(ref_traj(1,i) - ref_traj(1,i-1));
%     slope_real = (ref_traj(2,i) - state(2,i-1))/(ref_traj(1,i) - state(1,i-1));
%     if slope_real > slope_original
%     dirc = atan((ref_traj(2,i) - state(2,i-1))/(ref_traj(1,i) - state(1,i-1)));

    %%
    B = [cos(y(3,i)),0;
         sin(y(3,i)),0;
         0          ,1];


%     for j = num_steps-1:-1:1
%     %     P(:,:,i) = P(:,:,i+1) + dt*(A'*P(:,:,i+1) + P(:,:,i+1)*A - P(:,:,i+1)*B*R_inv*B'*P(:,:,i+1) + Q);
%         k = -inv(B'*b(:,:,j+1)*B+R)*B'*b(:,:,j+1)*A;
%         b(:,:,j) = A'*(b(:,:,j+1)-b(:,:,j+1)*B*inv(B'*b(:,:,j+1)*B+R)*B'*b(:,:,j+1))*A+Q;
%     %     s(:,i) = (A' - 0.5*(b(:,:,i)-Q)*B*R_inv*B')'*s(:,i+1) - Q*ref_traj(:,i+1);
%         s(:,j) = (A' + k'*B')*s(:,j+1) - Q*ref_traj(:,j+1);
%     %     s(:,i) = s(:,i+1) + dsdt*dt;
%     end
%     A*state(:,i-1)-ref_traj(:,i-1)

%     u(:,i-1) = -inv(R + B'*P(:,:,i)*B) * B'*P(:,:,i)*(A*state(:,i-1)-ref_traj(:,i-1));
%     move_s(u(1,i),u(2,i));
%     B = [cos(state(3,i-1)),0;
%          sin(state(3,i-1)),0;
%          0                ,1];

%     B = 0.01.*B;
%         P(:,:,j) = P(:,:,j+1) + dt*(A'*P(:,:,j+1) + P(:,:,j+1)*A - P(:,:,j+1)*B*R_inv*B'*P(:,:,j+1) + Q);
%         dsdt = (A' - P(:,:,j)*B*R_inv*B')'*s(:,j+1) - Q*ref_traj(:,j+1);
%         s(:,j) = s(:,j+1) + dsdt*dt;



%     u (:,i-1) = -inv(B'*b(:,:,i-1)*B+R)*B'*(b(:,:,i-1)*A*state(:,i-1)+s(:,i-1));
%     u(1,i) = ref_traj_db_dot(1,i) + kp1*(ref_traj(1,i)-y(1,i)) + kd1*(ref_traj_dot(1,i)-y(1,i)+y(1,i-1));
%     u(2,i) = ref_traj_db_dot(2,i) + kp2*(ref_traj(2,i)-y(2,i)) + kd2*(ref_traj_dot(2,i)-y(2,i)+y(2,i-1));
    u(1,i) = ref_traj_db_dot(1,i) + kp1*(ref_traj(1,i)-y(1,i)) + kd1*(ref_traj_dot(1,i)-y(1,i)+y(1,i-1));
    u(2,i) = ref_traj_db_dot(2,i) + kp2*(ref_traj(2,i)-y(2,i)) + kd2*(ref_traj_dot(2,i)-y(2,i)+y(2,i-1));
%     u(1,i-1) = ref_traj_dot(1,i-1) + kp1*(ref_traj(1,i-1)-x_hat(1,i-1));
%     u(2,i-1) = ref_traj_dot(2,i-1) + kp2*(ref_traj(2,i-1)-x_hat(2,i-1));
%     u(:,i-1) = -(inv(eye(2)+inv(R)*B'*P(:,:,i)*B)*inv(R)*B'*P(:,:,i)*A)*(state(:,i-1)-ref_traj(:,i-1));
%     u(:,i-1)=-0.5*inv(R)*B'*P(:,:,i-1)*state(:,i-1)-0.5*inv(R)*B'*s(:,i-1);
%     u(:,i-1)=dt*u(:,i-1);
%     state(:,i) = A*state(:,i-1) + B*u(:,i-1) + Theta*(ref_traj(:,i-1) - C*state(:,i-1));
    Xi = u(1,i)*cos(y(3,i))*dt+u(2,i)*sin(y(3,i))*dt
    omega = (u(2,i)*cos(y(3,i))-u(1,i)*sin(y(3,i)))/Xi
    if omega > 0.3
       omega = 0.3;
    elseif omega < -0.3
       omega = -0.3
    end
%     Xi = u(1,i-1)*cos(x_hat(3,i-1)) + u(2,i-1)*sin(x_hat(3,i-1));
%     omega = (u(2,i-1)*cos(x_hat(3,i-1))-u(1,i-1)*sin(x_hat(3,i-1)))/(dt*Xi);

    if y(3,i) > pi || y(3,i) < -pi
        y(3,i) = y(3,i) - 2*pi*sign(y(3,i));

    end
    error(i) = (norm(y(1:2,i)-ref_traj(1:2,i)))^2/2;
%     cost(i-1) = (state(:,i)-ref_traj(:,i))'*Q*(state(:,i)-ref_traj(:,i))+u(:,i)'*R*u(:,i);

    plot(ref_traj(1,i),ref_traj(2,i),'*')
    hold on

    msg.Angular.Z = omega;
    msg.Linear.X = Xi;
    send(robot,msg);

%     u = rdd(:,i-1)+(state(1:2,i-1)-ref_traj(1:2,i-1))
%     measurements(:,i) = A*measurements(:,i-1) + B*[Xi;omega];

    while toc<1
        plot(y(1,i),y(2,i),'.')
        hold on
        if strcmpi(get(gcf,'CurrentCharacter'),'e')
            rosshutdown;
            break;
        end
    end



    if strcmpi(get(gcf,'CurrentCharacter'),'e')
        rosshutdown;
        break;
    end
    toc
end
stop();
figure(2)
plot(y(1,:))
hold on
plot(ref_traj(1,:))

figure(3)
plot(y(2,:))
hold on
plot(ref_traj(2,:))

figure(4)
plot(y(3,:))
hold on
plot(ref_traj(3,:))

figure(5)
plot(error)

% rosshutdown;
