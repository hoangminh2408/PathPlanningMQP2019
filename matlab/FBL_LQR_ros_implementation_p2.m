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


%% Start
finish=false;
set(gcf,'CurrentCharacter','@'); % set to a dummy character


%% recalculate P and s
P = zeros([n,n,num_steps]);
s = zeros(n, num_steps);
for i = num_steps-1:-1:1
    P(:,:,i) = P(:,:,i+1) + dt*(A'*P(:,:,i+1) + P(:,:,i+1)*A - 0.5*P(:,:,i+1)*B*R_inv*B'*P(:,:,i+1) + 2*Q); % negative

    dsdt = (A' - 0.5*P(:,:,i+1)*B*R_inv*B')*s(:,i+1) - 2*Q*ref_traj(:,i+1);
    s(:,i) = s(:,i+1) + dsdt*dt;
end


%% reinitialize Phi%%
Phi = zeros(n,n,num_steps);
Phi_alpha = zeros(n,n,num_steps);
Theta = zeros(n,p,num_steps);
Theta_alpha = zeros(n,secureSensors,num_steps);
for i = 2:num_steps
    dPhi_alpha_dt = A*Phi_alpha(:,:,i-1) + Phi_alpha(:,:,i-1)*A' + Sigma_w - Phi_alpha(:,:,i-1)*C_alpha'*inv(Sigma_v_alpha)*C_alpha*Phi_alpha(:,:,i-1)';
    Phi_alpha(:,:,i) = Phi_alpha(:,:,i-1) + dt*dPhi_alpha_dt;
    Theta_alpha(:,:,i) = Phi_alpha(:,:,i)*C_alpha'*inv(Sigma_v_alpha);

    dPhi_dt = A*Phi(:,:,i-1) + Phi(:,:,i-1)*A' + Sigma_w - Phi(:,:,i-1)*C'*Sigma_v_inv*C*Phi(:,:,i-1)';
    Phi(:,:,i) = Phi(:,:,i-1) + dt*dPhi_dt;
    Theta(:,:,i) = Phi(:,:,i)*C'*Sigma_v_inv;
end


%%
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




state = receive(odom,3);
tbot_x = state.Pose.Pose.Position.X;
tbot_y = state.Pose.Pose.Position.Y;
quat = state.Pose.Pose.Orientation;
angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
scan = receive(laser);
ldata = readCartesian(scan);
y_lidar = sqrt(ldata(:,1).^2+ldata(:,2).^2);
y_dist(1,1) = min(y_lidar);
y_diff = y_dist(1,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% with redundant y coordinate measurement
y(:,1) = [tbot_x; tbot_y; y_dist(1,1) - y_diff; 0; 0];
y_alpha(:,1) = y(pMinusS,1);

% without redundant y coordinate measurement
% y(:,1) = [tbot_x; tbot_y; 0; 0];
% y_alpha(:,1) = y(pMinusS,1);

Xi = zeros(1,num_steps);
omega = zeros(1,num_steps);


u_ast(:,1) = u(:,1);
Xi(1,1) = 0 + dt*(u_ast(1,1)*cos(angles(1)) + u_ast(2,1)*sin(angles(1)));
omega(1,1) = (u_ast(2,1)*cos(angles(1))-u_ast(1,1)*sin(angles(1)))/Xi(1,1);

msg.Angular.Z = 0.1*omega(1,1);
msg.Linear.X = Xi(1,1);
send(robot,msg);
dx_hatdtarr = zeros(4,num_steps);
dxhat_alpha_dtarr = zeros(4,num_steps);
anglesarr = zeros(1,num_steps)
for i = 2:num_steps
    tic;

    %% predicting states for linearized system
    dx_hatdt = A*x_hat(:,i-1) + B*u_ast(:,i-1) + Theta(:,:,i-1)*(y(:,i-1) - C*x_hat(:,i-1));
    x_hat(:,i) = x_hat(:,i-1) + dt*dx_hatdt;
    
    dxhat_alpha_dt = A*x_alpha_hat(:,i-1) + B*u_ast(:,i-1) + Theta_alpha(:,:,i-1)*(y_alpha(:,i-1) - C_alpha*x_alpha_hat(:,i-1));
    x_alpha_hat(:,i) = x_alpha_hat(:,i-1) + dt*dxhat_alpha_dt;

    dx_hatdtarr(:,i) = dx_hatdt;
    dxhat_alpha_dtarr(:,i) = dxhat_alpha_dt;
    u (:,i) = -0.5*R_inv*B'*P(:,:,i)*x_alpha_hat(:,i) - 0.5*R_inv*B'*s(:,i);

    opt_start = tic;
    options = optimoptions(@fmincon,'Display','off','Algorithm','interior-point',...
        'SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true,...
        'HessianFcn',@(z,lambda)quadhess(z,lambda,2*R,2*eye(m)));
    constraint_matrix{1} = 2*eye(m);
    constraint_coefficient{1} = -2*u(:,i);
    constraint_constant{1} = u(:,i)'*u(:,i)-gm^2;
    fun = @(z)quadobj(z,2*R,B'*(P(:,:,i)*x_hat(:,i) + s(:,i)),0);
    nonlconstr = @(z)quadconstr(z,constraint_matrix,constraint_coefficient,constraint_constant);
    %x0 = [0;0]; % column vector

    [u_ast(:,i),fval,eflag,output,lambda] = fmincon(fun,u(:,1),...
        [],[],[],[],[],[],nonlconstr,options);
    opt_time = toc(opt_start);

%     u_diff(:,i) = u(:,i) - u_ast(:,i);
%     u_ast(:,i) = u(:,i);


    Xi(1,i) = Xi(1,i-1) + dt*(u_ast(1,i)*cos(angles(1)) + u_ast(2,i)*sin(angles(1)));
    omega(1,i) = (u_ast(2,i)*cos(angles(1))-u_ast(1,i)*sin(angles(1)))/Xi(1,i);

    msg.Angular.Z = omega(1,i);
    msg.Linear.X = Xi(1,i);
    send(robot,msg);

%     u = rdd(:,i-1)+(state(1:2,i-1)-ref_traj(1:2,i-1))
%     measurements(:,i) = A*measurements(:,i-1) + B*[Xi;omega];




    state = receive(odom,3);
    tbot_x = state.Pose.Pose.Position.X;
    tbot_y = state.Pose.Pose.Position.Y;
    quat = state.Pose.Pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    anglesarr(:,i) = angles(1);
    scan = receive(laser);
    ldata = readCartesian(scan);
    y_lidar = sqrt(ldata(:,1).^2+ldata(:,2).^2);
    y_dist(1,i) = min(y_lidar);

    %% calcultate Z error
    a = 0.01*randn;
%     a = 0;

    y(1,i) = tbot_x;
    y(2,i) = tbot_y;

    % with redundant y coordinate measurement
%     y(3,i) = y_dist(1,i) - y_diff + a;
    y(3,i) = y_dist(1,i) - y_diff;
%     y(3,i) = tbot_y;
    y(4,i) = (tbot_x - y(1,i-1))/dt;
    y(5,i) = (tbot_y - y(2,i-1))/dt;

    % without redundant y coordinate measurement
%     y(3,i) = (tbot_x - y(1,i-1))/dt;
%     y(4,i) = (tbot_y - y(2,i-1))/dt;


    y_alpha(:,i) = y(pMinusS,i);


     while toc<dt
%         plot(y(1,i),y(2,i),'.')
%         hold on
        if strcmpi(get(gcf,'CurrentCharacter'),'e')
            rosshutdown;
            break;
        end
    end



    if strcmpi(get(gcf,'CurrentCharacter'),'e')
        rosshutdown;
        break;
    end
    toc;



end
stop()
% plot(ref_traj(1,:),ref_traj(2,:),'*')
figure(1)
plot(ref_traj(1,:),ref_traj(2,:))
hold on
plot(y(1,:),y(2,:),'.')



figure(2)
plot(y(1,:))
hold on
plot(ref_traj(1,:))

figure(3)
plot(y(2,:))
hold on
plot(ref_traj(2,:))

figure(4)
plot(y(2,:))
hold on
plot(y_dist-y_diff)

for i=1:num_steps
    error(i) = (norm(x_hat(1:2,i)-ref_traj(1:2,i)))^2/2;
end
figure(5)
plot(error)
% rosshutdown;

diff = ref_traj(2,:) - (y_dist - y_diff);
var = sum(diff.^2)/num_steps
