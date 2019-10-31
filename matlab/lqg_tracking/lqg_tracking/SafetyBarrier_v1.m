function [sol,q] = SafetyBarrier_v1(A,B,C_alpha,Sigma_w,Sigma_v,Sigma_v_alpha,R,K,Phi,Theta,n,m,p,gam,s_coeff, degree, start_point, g_U, g_D)


var = [];
state_var = [];
x = [];
u = [];
x_var = [];
syms t;

for i = 1:n
    eval(sprintf('syms x%d', i));
    eval(sprintf('syms u%d', i));
    eval(sprintf('syms x_alpha_%d', i));
    
    eval(sprintf('var = [var; x%d]', i));
    eval(sprintf('var = [var; u%d]', i));
    eval(sprintf('state_var = [state_var; x%d]', i));
    eval(sprintf('x = [x; x%d]', i));
    eval(sprintf('u = [u; u%d]', i));
    eval(sprintf('x_var = [x_var; x%d]', i));
    eval(sprintf('x_var = [x_var; x_alpha_%d]', i));
end

var = [var; t];
state_var = [state_var; t];



temp = -1*inv(R)*B';

% syms x1 x2 x_alpha_1 x_alpha_2 u1 u2 t;
%x0 = [0;0];
x0 = start_point;
epsilon = 1;
% var = [x1; x2; u1; u2; t];
% state_var = [x1;x2;t];
init_x_var = [x0;t];
%x_var = [x1; x2; x_alpha_1; x_alpha_2];
% x = [x1;x2];
% x_alpha = [x_alpha_1;x_alpha_2];


% target = 5;
% obs = 2.5;
% 
% g_U = 1 - (x1)^2 + (x2-obs)^2;
% g_D = 1 - (x1)^2 + (x2-target)^2;
%%%%%%%%
%s_coeff_1 = [0,0,1];
%s_coeff_2 = [0,0,1];
%degree = 1;
% s_poly_1 = 0;
% s_poly_2 = 0;
s_appx = double.empty(0,1);
for i = 1:1:n
    for j = 1:1:(degree+1)
        tmp = 0;
%     s_poly_1 = s_poly_1 + s_coeff(i)*t^(degree-i+1);
%     s_poly_2 = s_poly_2 + s_coeff(i)*t^(degree-i+1);
        tmp = tmp + s_coeff(i,j)*t^(degree - j + 1);
    end
    s_appx = [s_appx; tmp];
end
% s_appx = [s_poly_1;s_poly_2];
% u_hat_1 = K(1,:)*x_alpha-temp(1,:)*s_appx + u1;
% u_hat_2 = K(2,:)*x_alpha-temp(2,:)*s_appx + u2;
% u_hat = [u_hat_1;u_hat_2];
%%%%%%%%
degree = 2;

% A1 = Phi*C_alpha'*Sigma_v_alpha*C_alpha;
% A2 = A-Theta*C_alpha+B*K;
%f = [A(1,1)*x1+A(1,2)*x2+B(1,:)*u_hat;A(2,1)*x1+A(2,2)*x2+B(2,:)*u_hat;A1(1,1)*x1+A1(1,2)*x2+A2(1,1)*x_alpha_1+A2(1,2)*x_alpha_2+B(1,:)*u_hat;A1(2,1)*x1+A1(2,2)*x2+A2(2,1)*x_alpha_1+A2(2,2)*x_alpha_2+B(2,:)*u_hat];
% f1 = A*x+B*K*x_alpha + B*u_hat;
% f2 = Phi*C_alpha'*Sigma_v_alpha*C_alpha*x + (A-Theta*C_alpha+B*K)*x_alpha;
%f = [u1 - t + x1 - 0.5*x_alpha_1;u2 - t + x2 - 0.5*x_alpha_2;u1 - t + x1 - x_alpha_1 - 0.5*x_alpha_2;u2-t];


% B_bar = [B;B];
% lamda = [[eye(n);zeros(n)]*sqrt(Sigma_w), [zeros(n);[Theta Theta]]*sqrt(Sigma_v)];
lamda = [[eye(n);zeros(n)]*sqrt(Sigma_w), [zeros(n,size(Theta,2));Theta]*sqrt(Sigma_v_alpha)];
% g_D_gamma = (gam^2-u1^2 - u2^2);

prog = sosprogram(var);
[prog,D] = sospolyvar(prog, monomials(state_var,1:2),'wscoeff');    % constraint (12) incorporated
% prog
% test = monomials(init_x_var,1)
%[prog,D_init] = sospolyvar(prog,monomials(init_x_var,1)); % constraint (10)
[prog,D_init] = sospolyvar(prog,monomials(state_var,1)); % constraint (10)
% D_init
D_init = subs(D_init, state_var, init_x_var);
% D_init
% state_var
% init_x_var
prog = sosineq(prog,epsilon - D_init); % constraint (10)
[prog,lambda_D] = sossosvar(prog, monomials(var,1:degree)); % constraint (14) incorporated
[prog,lambda_U] = sossosvar(prog, monomials(var,1:degree));

dd = sym(zeros(2*n));
for i = 1:(2*n)
    for j = 1:(2*n)
        dd(i,j) = diff(diff(D,x_var(i)),x_var(j));
    end
end
% dd = [diff(diff(D,x1),x1), diff(diff(D,x1),x2), diff(diff(D,x1),x_alpha_1),diff(diff(D,x1),x_alpha_2);diff(diff(D,x2),x1),diff(diff(D,x2),x2),diff(diff(D,x2),x_alpha_1),diff(diff(D,x2),x_alpha_2);diff(diff(D,x_alpha_1),x1), diff(diff(D,x_alpha_1),x2), diff(diff(D,x_alpha_1),x_alpha_1),diff(diff(D,x_alpha_1),x_alpha_2);diff(diff(D,x_alpha_2),x1),diff(diff(D,x_alpha_2),x2),diff(diff(D,x_alpha_2),x_alpha_1),diff(diff(D,x_alpha_2),x_alpha_2)];
% d1 = [diff(D,x1);diff(D,x2)];
% d2 = [diff(D,x_alpha_1);diff(D,x_alpha_2)];
% dddd = [diff(diff(D,x1),x1) diff(diff(D,x1),x2); diff(diff(D,x2),x1) diff(diff(D,x2),x2)];
prog = sosineq(prog, (-1+D-lambda_U*g_U)); % constraint (11)
%expr = -d1(1)*f(1)+d1(2)*f(2)-d2(1)*f(3)-d2(2)*f(4)-lambda_D*g_D_gamma-diff(D,t)-0.5*trace(lamda'*dd*lamda);
%expr = -diff(D,t)-0.5*trace(lamda'*dd*lamda);
%prog = sosineq(prog,expr); % constraint (14)

dddx = sym(zeros(n,1));
for i = 1:n
    i
    -diff(D,x(i))
    K(i,:)
    x
    temp(i,:)
    s_appx
    u(i)
    dddx(i,1) = (-diff(D,x(i))*(K(i,:)*x + temp(i,:)*s_appx + u(i)));
end
% sum(dddx)
% diff(D,t) 
% lambda_D*g_D 
% lamda
% dd
% 0.5*trace(lamda'*dd*lamda)
prog = sosineq(prog,sum(dddx) - lambda_D*g_D - diff(D,t) - 0.5*trace(lamda'*dd*lamda));
% prog = sosineq(prog,-diff(D,x1)*(K(1,:)*x + temp(1,:)*[s_appx(1); s_appx(2)] + u1) - diff(D,x2)*(K(2,:)*x + temp(2,:)*[s_appx(1); s_appx(2)] + u2) - lambda_D*g_D-diff(D,t)-0.5*trace(lamda'*dd*lamda));
option.solver = 'sdpt3';

[prog,info] = sossolve(prog,option);
sol = sosgetsol(prog,D);
if info.pinf == 1
    q = 1;
else
    q = 0;
end