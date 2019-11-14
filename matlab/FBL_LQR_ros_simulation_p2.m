% %% turtlebot
% % rosinit;
% robot = rospublisher('/cmd_vel');
% laser = rossubscriber('/scan');
% imu = rossubscriber('/imu');
% odom = rossubscriber('/odom');
% stmsg = rosmessage(odom);
% msg = rosmessage(robot);
% 
% %% State receive
% 
% % state.Pose.Pose.Orientation
% 
% 
% %% Start
% finish=false;
% set(gcf,'CurrentCharacter','@'); % set to a dummy character
    
    



%% reinitialize Phi%%
Phi = zeros(n,n,num_steps); 
Theta = zeros(n,p,num_steps);

Phi_r1 = zeros(n,n,num_steps); 
Theta_r1 = zeros(n,size(F_r1_comp,2),num_steps);

Phi_r2 = zeros(n,n,num_steps); 
Theta_r2 = zeros(n,size(F_r2_comp,2),num_steps);

Phi_r1r2 = zeros(n,n,num_steps); 
Theta_r1r2 = zeros(n,size(F_r1r2_comp,2),num_steps);

for i = 2:num_steps
    dPhi_dt = A*Phi(:,:,i-1) + Phi(:,:,i-1)*A' + Sigma_w - Phi(:,:,i-1)*C'*Sigma_v_inv*C*Phi(:,:,i-1)';
    Phi(:,:,i) = Phi(:,:,i-1) + dt*dPhi_dt;
    Theta(:,:,i) = Phi(:,:,i)*C'*Sigma_v_inv;
    
    dPhi_r1dt = A*Phi_r1(:,:,i-1) + Phi_r1(:,:,i-1)*A' + Sigma_w - Phi_r1(:,:,i-1)*C_r1'*Sigma_v_r1inv*C_r1*Phi_r1(:,:,i-1)';
    Phi_r1(:,:,i) = Phi_r1(:,:,i-1) + dt*dPhi_r1dt;
    Theta_r1(:,:,i) = Phi_r1(:,:,i)*C_r1'*Sigma_v_r1inv;
    
    dPhi_r2dt = A*Phi_r2(:,:,i-1) + Phi_r2(:,:,i-1)*A' + Sigma_w - Phi_r2(:,:,i-1)*C_r2'*Sigma_v_r2inv*C_r2*Phi_r2(:,:,i-1)';
    Phi_r2(:,:,i) = Phi_r2(:,:,i-1) + dt*dPhi_r2dt;
    Theta_r2(:,:,i) = Phi_r2(:,:,i)*C_r2'*Sigma_v_r2inv;
    
    dPhi_r1r2dt = A*Phi_r1r2(:,:,i-1) + Phi_r1r2(:,:,i-1)*A' + Sigma_w - Phi_r1r2(:,:,i-1)*C_r1r2'*Sigma_v_r1r2inv*C_r1r2*Phi_r1r2(:,:,i-1)';
    Phi_r1r2(:,:,i) = Phi_r1r2(:,:,i-1) + dt*dPhi_r1r2dt;
    Theta_r1r2(:,:,i) = Phi_r1r2(:,:,i)*C_r1r2'*Sigma_v_r1r2inv;
end


%%
% first_step_angle = atan((ref_traj(2,2) - ref_traj(2,1))/(ref_traj(1,2) - ref_traj(1,1)));
% init_angle = 0;
% theta = first_step_angle-init_angle(1);
% 
% state_init = [0;0;1e-4];
% 
% B_ind = 0; 




% state = receive(odom,3);
% tbot_x = state.Pose.Pose.Position.X;
% tbot_y = state.Pose.Pose.Position.Y;
% quat = state.Pose.Pose.Orientation;
% angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% scan = receive(laser);
% ldata = readCartesian(scan);
% y_lidar = sqrt(ldata(:,1).^2+ldata(:,2).^2);
% y_dist(1,1) = min(y_lidar);
% y_diff = y_dist(1,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% with redundant y coordinate measurement
% y(:,1) = [tbot_x; tbot_y; y_dist(1,1) - y_diff; 0; 0];
% y_alpha(:,1) = y(pMinusS,1);

% without redundant y coordinate measurement
% y(:,1) = [tbot_x; tbot_y; 0; 0];
% y_alpha(:,1) = y(pMinusS,1);

Xi = zeros(1,num_steps);
omega = zeros(1,num_steps);
% 
% 
% u (:,1) = -0.5*R_inv*B'*P(:,:,1)*x_hat(:,1) - 0.5*R_inv*B'*s(:,1);
% u_ast(:,1) = u(:,1);

% options = optimoptions(@fmincon,'Display','off','Algorithm','interior-point',...
%         'SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true,...
%         'HessianFcn',@(z,lambda)quadhess(z,lambda,2*R,2*eye(m)));
% constraint_matrix{1} = 2*eye(m);
% constraint_coefficient{1} = -2*u(:,i);
% constraint_constant{1} = u(:,i)'*u(:,i)-gm^2;
% fun = @(z)quadobj(z,2*R,B'*(P(:,:,i)*x_hat(:,i) + s(:,i)),0);
% nonlconstr = @(z)quadconstr(z,constraint_matrix,constraint_coefficient,constraint_constant);
% %x0 = [0;0]; % column vector
% [u_ast(:,i),fval,eflag,output,lambda] = fmincon(fun,u(:,1),...
%     [],[],[],[],[],[],nonlconstr,options);

% Xi(1,1) = 0 + dt*(u_ast(1,1)*cos(angles(1)) + u_ast(2,1)*sin(angles(1)));
% omega(1,1) = (u_ast(2,1)*cos(angles(1))-u_ast(1,1)*sin(angles(1)))/Xi(1,1);

Xi(1,1) = 0;
omega(1,1) = 0;

orientation(1,1) = dt*omega(1,1);

% msg.Angular.Z = 0.1*omega(1,1);
% msg.Linear.X = Xi(1,1);
% send(robot,msg);
is_constr_r1 = 1;
is_constr_r2 = 1;
for i = 2:num_steps
    tic;
    
    %% predicting states for linearized system
    dx_hatdt = A*x_hat(:,i-1) + B*u_ast(:,i-1) + Theta(:,:,i-1)*(y(:,i-1) - C*x_hat(:,i-1));
    x_hat(:,i) = x_hat(:,i-1) + dt*dx_hatdt;
    
    %% r1
    dx_hat_r1dt = A*x_hat_r1(:,i-1) + B*u_ast(:,i-1) + Theta_r1(:,:,i-1)*(y_r1(:,i-1) - C_r1*x_hat_r1(:,i-1));
    x_hat_r1(:,i) = x_hat_r1(:,i-1) + dt*dx_hat_r1dt;
    
    %% r2
    dx_hat_r2dt = A*x_hat_r2(:,i-1) + B*u_ast(:,i-1) + Theta_r2(:,:,i-1)*(y_r2(:,i-1) - C_r2*x_hat_r2(:,i-1));
    x_hat_r2(:,i) = x_hat_r2(:,i-1) + dt*dx_hat_r2dt;
    
    %% r1&r2
    dx_hat_r1r2dt = A*x_hat_r1r2(:,i-1) + B*u_ast(:,i-1) + Theta_r1r2(:,:,i-1)*(y_r1r2(:,i-1) - C_r1r2*x_hat_r1r2(:,i-1));
    x_hat_r1r2(:,i) = x_hat_r1r2(:,i-1) + dt*dx_hat_r1r2dt;
    
    %% original WMR system
    
    
    
    %% measurements
    y(:,i) = C*x_hat(:,i);
    y_r1(:,i) = C_r1*x_hat_r1(:,i);
    y_r2(:,i) = C_r2*x_hat_r2(:,i);
    y_r1r2(:,i) = C_r1r2*x_hat_r1r2(:,i);
    
    %% Contraint Generator
    % CLF
    
    
    % CBF
    
    
    
    %% Controller
%     opt_start = tic;
    options = optimoptions(@fmincon,'Display','off','Algorithm','interior-point',...
        'SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true,...
        'HessianFcn',@(z,lambda)quadhess(z,lambda,2*R,0));
    constraint_matrix1 = 2*[0 1/(x_hat_r1(2,i)+0.15)^2 0 0]*B;
    constraint_matrix2 = 2*[0 1/(x_hat_r2(2,i)+0.15)^2 0 0]*B;
    constraint_matrix3 = 2*(x_hat_r1(:,i)-target)'*B;
    constraint_matrix4 = 2*(x_hat_r2(:,i)-target)'*B;
    H_B_r1 = [0 0 0 0;
              0 -2*(x_hat_r1(2,i)+0.15)^(-3) 0 0;
              0 0 0 0;
              0 0 0 0];
    H_B_r2 = [0 0 0 0;
              0 -2*(x_hat_r2(2,i)+0.15)^(-3) 0 0;
              0 0 0 0;
              0 0 0 0];
    constraint_constant1 = -x_hat_r1(2,i) - 0.15 - gm*norm([0 1/(x_hat_r1(2,i)+0.15)^2 0 0]*Theta_r1(:,:,i-1)*C_r1) - 0.5*trace(sqrtm(Sigma_v_r1)'*Theta_r1(:,:,i-1)'*H_B_r1*Theta_r1(:,:,i-1)*sqrtm(Sigma_v_r1));
    constraint_constant2 = -x_hat_r2(2,i) - 0.15 - gm*norm([0 1/(x_hat_r2(2,i)+0.15)^2 0 0]*Theta_r2(:,:,i-1)*C_r1) - 0.5*trace(sqrtm(Sigma_v_r2)'*Theta_r2(:,:,i-1)'*H_B_r2*Theta_r2(:,:,i-1)*sqrtm(Sigma_v_r2));
    constraint_constant3 = (x_hat_r1(:,i)-target)'*A*x_hat_r1(:,i);
    constraint_constant4 = (x_hat_r2(:,i)-target)'*A*x_hat_r2(:,i);
    constraint_matrix = [constraint_matrix1; constraint_matrix2; constraint_matrix3; constraint_matrix4];
    constraint_constant = [constraint_constant1; constraint_constant2; constraint_constant3; constraint_constant4];
    fun = @(z)quadobj(z,2*R,[0;0],0);
    
    %x0 = [0;0]; % column vector
%     objf = @(u) u'*u
    [u_ast(:,i),fval,eflag,output,lambda]=fmincon(fun,u(:,1),constraint_matrix,constraint_constant,[],[],[],[],[],options);
%     [u_ast(:,i),fval,eflag,output,lambda] = fmincon(fun,u(:,1),...
%         constraint_matrix,constraint_constant,[],[],[],[],[],options);
%     opt_time = toc(opt_start);

%     u_diff(:,i) = u(:,i) - u_ast(:,i);
%     u_ast(:,i) = u(:,i);
    
    if eflag == 1
        Xi(1,i) = Xi(1,i-1) + dt*(u_ast(1,i)*cos(orientation(1,i-1)) + u_ast(2,i)*sin(orientation(1,i-1)));
        omega(1,i) = (u_ast(2,i)*cos(orientation(1,i-1))-u_ast(1,i)*sin(orientation(1,i-1)))/Xi(1,i);
        orientation(1,i) = orientation(1,i-1) + dt*omega(1,i);
        continue
    elseif eflag == -2
        if norm(x_hat_r1 - x_hat_r2) > threshold
            if norm(x_hat_r2 - x_hat_r1r2) > (threshold/2) && norm(x_hat_r1 - x_hat_r1r2) > (threshold/2)
                constraint_matrix = [];
                constraint_constant = [];
                is_constr_r1 = 0;
                is_constr_r2 = 0;
            
            elseif norm(x_hat_r1 - x_hat_r1r2) > (threshold/2)
                constraint_matrix = [constraint_matrix2; constraint_matrix4];
                constraint_constant = [constraint_constant2; constraint_constant4];
                is_constr_r1 = 0;
            elseif norm(x_hat_r2 - x_hat_r1r2) > (threshold/2)
                constraint_matrix = [constraint_matrix1; constraint_matrix3];
                constraint_constant = [constraint_constant1; constraint_constant3];
                is_constr_r2 = 0;
            end
        end
    end
    
    [u_ast(:,i),fval,eflag,output,lambda] = fmincon(fun,u(:,1),...
        constraint_matrix,constraint_constant,[],[],[],[],[],options);
    
    if eflag == 1
        Xi(1,i) = Xi(1,i-1) + dt*(u_ast(1,i)*cos(orientation(1,i-1)) + u_ast(2,i)*sin(orientation(1,i-1)));
        omega(1,i) = (u_ast(2,i)*cos(orientation(1,i-1))-u_ast(1,i)*sin(orientation(1,i-1)))/Xi(1,i);
        orientation(1,i) = orientation(1,i-1) + dt*omega(1,i);
        continue
    elseif eflag == -2
        while (is_constr_r1 + is_constr_r2)
            if norm(y_r1(:,i) - x_hat_r1(:,i)) > norm(y_r2(:,i) - x_hat_r2(:,i))
                constraint_matrix = [constraint_matrix2; constraint_matrix4];
                constraint_constant = [constraint_constant2; constraint_constant4];
                is_constr_r1 = 0;
                y_r1(:,i) = x_hat_r1(:,i);    
            else
                constraint_matrix = [constraint_matrix1; constraint_matrix3];
                constraint_constant = [constraint_constant1; constraint_constant3];
                is_constr_r2 = 0;
                y_r2(:,i) = x_hat_r2(:,i);
            end
        
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
        %     opt_time = toc(opt_start);
        end
    end
        
        
    
    Xi(1,i) = Xi(1,i-1) + dt*(u_ast(1,i)*cos(orientation(1,i-1)) + u_ast(2,i)*sin(orientation(1,i-1)));
    omega(1,i) = (u_ast(2,i)*cos(orientation(1,i-1))-u_ast(1,i)*sin(orientation(1,i-1)))/Xi(1,i);
    orientation(1,i) = orientation(1,i-1) + dt*omega(1,i);
    
%     msg.Angular.Z = omega(1,i);
%     msg.Linear.X = Xi(1,i);
%     send(robot,msg);
   
%     state = receive(odom,3);
%     tbot_x = state.Pose.Pose.Position.X;
%     tbot_y = state.Pose.Pose.Position.Y;
%     quat = state.Pose.Pose.Orientation;
%     angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    
%     scan = receive(laser);
%     ldata = readCartesian(scan);
%     y_lidar = sqrt(ldata(:,1).^2+ldata(:,2).^2);
%     y_dist(1,i) = min(y_lidar);

    %% calcultate Z error 
%     a = 0.01*randn;
% %     a = 0;
%     
%     y(1,i) = tbot_x;
%     y(2,i) = tbot_y;
%     
%     % with redundant y coordinate measurement
%     y(3,i) = y_dist(1,i) - y_diff + a;
% %     y(3,i) = tbot_y;
%     y(4,i) = (tbot_x - y(1,i-1))/dt;
%     y(5,i) = (tbot_y - y(2,i-1))/dt;
%     
%     % without redundant y coordinate measurement
% %     y(3,i) = (tbot_x - y(1,i-1))/dt;
% %     y(4,i) = (tbot_y - y(2,i-1))/dt;
%     
%     
%     y_alpha(:,i) = y(pMinusS,i);
%     
%     
%      while toc<dt
% %         plot(y(1,i),y(2,i),'.')
% %         hold on
%         if strcmpi(get(gcf,'CurrentCharacter'),'e')
%             rosshutdown;
%             break;
%         end
%     end
%     
%     
%     
%     if strcmpi(get(gcf,'CurrentCharacter'),'e')
%         rosshutdown;
%         break;
%     end
%     toc;
    
    
    
end
stop()
% plot(ref_traj(1,:),ref_traj(2,:),'*')
figure(1)
% plot(ref_traj(1,:),ref_traj(2,:))
% hold on
plot(y(1,:),y(2,:),'.')



% figure(2)
% plot(y(1,:))
% hold on
% plot(ref_traj(1,:))
% 
% figure(3)
% plot(y(2,:))
% hold on
% plot(ref_traj(2,:))
% 
% figure(4)
% plot(y(2,:))
% hold on
% plot(y_dist-y_diff)
% 
% for i=1:num_steps
%     error(i) = (norm(x_hat(1:2,i)-ref_traj(1:2,i)))^2/2;
% end
% figure(5)
% plot(error)
% % rosshutdown;
% 
% diff = ref_traj(2,:) - (y_dist - y_diff);
% var = sum(diff.^2)/num_steps