function u_ast = QCQPSolver(B,R,u_alpha,gamma,P,x_hat,s,i,n,start_point)
 opt_start = tic;
 options = optimoptions(@fmincon,'Display','off','Algorithm','interior-point',...
 'SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true,...
 'HessianFcn',@(z,lambda)quadhess(z,lambda,2*R,2*eye(n)));
 disp(-2*u_alpha)
 constraint_matrix{1} = eye(n);
 constraint_coefficient{1} = -2*u_alpha;
 constraint_constant{1} = u_alpha'*u_alpha-gamma^2;
 fun = @(z)quadobj(z,2*R,B'*(2*P(:,:,i-1)*x_hat(:,i-1) + 2*s(:,i)),0);
 nonlconstr = @(z)quadconstr(z,constraint_matrix,constraint_coefficient,constraint_constant);
 %x0 = [0;0]; % column vector
 x0 = start_point;
 [u_ast,fval,eflag,output,lambda] = fmincon(fun,x0,...
 [],[],[],[],[],[],nonlconstr,options);
 opt_time = toc(opt_start);
end
