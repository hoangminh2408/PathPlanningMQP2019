function hess = quadhess(x,lambda,Q,H)
hess = Q;
jj = 1; % jj is the number of inequality constraints
for i = 1:jj
    hess = hess + lambda.ineqnonlin(i)*H;
end