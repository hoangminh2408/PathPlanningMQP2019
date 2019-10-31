function gam = InvokeSafetyBarrier(A,B,C_alpha,Sigma_w,Sigma_v,Sigma_v_alpha,R,K,Phi,Theta,n,m,p,s_coeff,degree,start_point,g_U,g_D)


%%%% system setup %%%%

% n = 2;
% A = zeros(n);
% B = eye(n);
% C = eye(n);
%Q = eye(n);
% R = eye(n);
%Sigma_w = eye(n);
%Sigma_v = eye(n);
% T = 1;
%sys = ss(A,B,C,0);

%%%% Compute \gamma %%%%%

%Theta = eye(2);
% P = eye(2);
%K = eye(2);
gamma_max = 10;
rho = 0.1;
gamma_lb = 0;
gamma_ub = gamma_max;
while (gamma_ub - gamma_lb > rho)
    gam = (gamma_lb + gamma_ub)/2;
    [~,q] = SafetyBarrier_v1(A,B,C_alpha,Sigma_w,Sigma_v,Sigma_v_alpha,R,K,Phi,Theta,n,m,p,gam,s_coeff,degree,start_point,g_U,g_D);
    if q == 0
        gamma_ub = gam;
    else
        gamma_lb = gam;
    end
end

end

