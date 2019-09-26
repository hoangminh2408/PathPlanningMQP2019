%% Smooth Move function
function move_s(input_x, input_y, dt)
    %% Default Settings
    if nargin<3
        dt = 1;
    end

    %% ROS Config
    robot = rospublisher('/cmd_vel');
    odom = rossubscriber('/odom');
    stmsg = rosmessage(odom);
    msg = rosmessage(robot);
    set(gcf,'CurrentCharacter','@'); % set to a dummy character
    %% Plot
    figure(1)
    state = receive(odom,3);
    tbot_x = state.Pose.Pose.Position.X;
    tbot_y = state.Pose.Pose.Position.Y;
    plot(tbot_x,tbot_y,'.')
    hold on


    %% Compute Curve Parameters
    quat = state.Pose.Pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    if input_x>0
        actangle = atan(input_y/input_x);
    else
        if input_y>0
            actangle = atan(input_y/input_x)+pi;
        else
            actangle = atan(input_y/input_x)-pi;
        end
    end
    theta0 = angles(1);
    if actangle > theta0
        theta1 = theta0+2*actangle;
    else
        theta1 = theta0-2*actangle;
    end
    a0 = tbot_x;
    a1 = 2*(tan(theta1)*input_x - input_y)/(tan(theta1)-tan(theta0))
    a2 = input_x - a1
    b0 = tbot_y;
    b1 = a1*tan(theta0)
    b2 = input_y - b1

    A = 4*a2^2 + 4*b2^2
    B = 4*a1*a2 + 4*b1*b2
    C = a1^2+b1^2

    fun = @(x) sqrt(A.*x.^2 + B.*x +C)
    dl_hat = integral(fun,0,1)
    if norm(dl_hat) < norm([input_x,input_y])
        disp('Something is wrong')
        disp(norm([input_x,input_y]))
    end
%     disp(step);
%     figure(2)
%     plot(step,A)
%     hold on
%     figure(3)
%     plot(step,B)
%     hold on
%     figure(4)
%     plot(step,C)
%     hold on
%     figure(5)
%     plot(step,dl_hat)
%     hold on
    %% Move
    theta = actangle-angles(1)
    msg.Angular.Z = 2*theta;
%     msg.Linear.X = norm(input_x,input_y);
    msg.Linear.X = dl_hat;
    send(robot,msg);
    % Running for delta t seconds
    tic
    while toc<dt
        state = receive(odom,3);
        tbot_x = state.Pose.Pose.Position.X;
        tbot_y = state.Pose.Pose.Position.Y;
        plot(tbot_x,tbot_y,'.')
        if strcmpi(get(gcf,'CurrentCharacter'),'e')
            stop;
            break;
        end
    end
end
