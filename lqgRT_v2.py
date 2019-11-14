#!/usr/bin/env python
import numpy as np
import math
import copy
import time
# import matplotlib.pyplot as plt
import time
import matlab.engine
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios


# T              -- the final time of the system
# dt             -- the time duration of each iteration
# num_steps      -- the number of iterations during T, 1000 is not enough
# n              -- the dimension of the system state
# m              -- the dimension of the system input
# p              -- the dimension of the system observation
# pMinusS        -- the index list of the secure sensors(row vector)
# A              -- n*n state matrix
# B              -- n*m input matrix
# C              -- p*n output matrix
# C_alpha        -- the matrix obtained by selecting the rows from C indexed in
#                   the observation matrix y that are not affected by the adversary
# Q              -- n*n cost matrix
# R              -- m*m cost matrix
# Sigma_w        -- n*n autocorrelation matrix
# Sigma_v        -- p*p autocorrelation matrix
# Sigma_v_alpha  -- the covariance matrix of v_alpha
# ref_traj       -- n*num_steps polynomial reference trajectory
# degree         -- the degree of the polyfit for the reference trajectory
eng = matlab.engine.start_matlab()
def lqgRT_v2(T, num_steps, n, m, p, pMinusS, A, B, C, Sigma_w, Sigma_v, Q, R, start_point, rd_tar, rd_obs, target, obs, t, parametric_func, degree):
    dt = T/num_steps
    g_D = math.pow(rd_tar,2)
    g_U = math.pow(rd_obs,2)

    ref_traj = parametric_func
    s_coeff = np.zeros((n,degree + 1))
    for i in range(0,n):
        s_coeff[i,:] = np.polyfit(t, np.reshape(ref_traj[i,:],num_steps), degree)
    for i in range(0,n):
        ref_traj[i,:] = np.polyval(s_coeff[i,:], t)
    if dt <= 0:
        dt = 1e-4
    if n <= 0:
        n = 2
    if m <= 0:
        m = 2
    if p <= 0:
        p = 2
    secureSensors = len(pMinusS)
    C_alpha = C[pMinusS-2,:]
    Sigma_v_alpha = Sigma_v[pMinusS-2, pMinusS-2]
    R_inv = np.linalg.inv(R)
    P = np.zeros((n,n,num_steps))

    x = np.zeros((n, num_steps))
    s = np.zeros((n, num_steps))
    x_hat = np.zeros((n, num_steps))
    x_alpha_hat = np.zeros((n,num_steps))
    x_real = np.zeros((n,num_steps))
    x0 = start_point
    x[:,0] = np.reshape(x0,2)
    x_hat[:,0] = np.reshape(x0,2)
    x_alpha_hat[:,0] = np.reshape(x0,2)
    x_real[:,0] = np.reshape(x0,2)

    G = Sigma_w;
    Sigma_v_inv = np.linalg.inv(Sigma_v)

    Phi = np.zeros((n,n));
    Theta = np.zeros((n, p));
    Phi_alpha = np.zeros((n,n))
    Theta_alpha = np.zeros((n, secureSensors))

    A_h = A.conj().transpose()
    B_h = B.conj().transpose()
    C_h = C.conj().transpose()
    C_alpha_h = C_alpha.conj().transpose()
    Phi_alpha_h = Phi_alpha.conj().transpose()
    for i in range(num_steps - 2, -1, -1):
        P[:,:,i] = P[:,:,i+1] + dt*(np.matmul(A_h,P[:,:,i+1]) + np.matmul(P[:,:,i+1],A) - np.matmul(np.matmul(np.matmul(np.matmul(P[:,:,i+1],B),R_inv),B_h),P[:,:,i+1]) + Q)
        dsdt = (A_h - np.matmul(np.matmul(np.matmul(P[:,:,i],B),R_inv),B_h)).conj().transpose()
        dsdt = np.matmul(dsdt, s[:,i+1]) - np.matmul(Q,ref_traj[:,i+1])
        s[:,i] = s[:,i+1] + dsdt*dt
    BG = np.hstack((B,G))
    K = np.asarray(eng.KalmanOutput(matlab.double(A.tolist()), matlab.double(BG.tolist()), matlab.double(C.tolist()), 0, matlab.double(Q.tolist()), matlab.double(R.tolist()), 0, nargout = 1))

    Phi_alpha_prev = Phi_alpha + 0.0001
    Theta_alpha_prev = Theta_alpha + 0.0001
    Phi_alpha_bool =  abs(Phi_alpha - Phi_alpha_prev) >= 0.001
    while np.all(abs(Phi_alpha - Phi_alpha_prev) >= 0.001) or np.all(abs(Theta_alpha_prev - Theta_alpha_prev) >= 0.001):
        Phi_alpha_prev = Phi_alpha_prev
        Theta_alpha_prev = Theta_alpha
        dPhi_alpha_dt = np.matmul(A,Phi_alpha) + np.matmul(Phi_alpha,A_h) + Sigma_w - np.matmul(np.matmul(np.matmul(np.matmul(Phi_alpha, C_alpha_h),np.linalg.inv(Sigma_v_alpha)),C_alpha),Phi_alpha_h)
        Phi_alpha = Phi_alpha + dt*dPhi_alpha_dt
        Theta_alpha = np.matmul(np.matmul(Phi_alpha, C_alpha_h), np.linalg.inv(Sigma_v_alpha))

    gamma = 9.9216
    Phi = np.zeros((n,n))
    Phi_alpha = np.zeros((n,n))
    Phi_alpha_h = Phi_alpha.conj().transpose()
    for i in range(1, num_steps):
        u_alpha = np.matmul(np.matmul(np.matmul(-0.5*R_inv,B_h),P[:,:,i-1]),x_alpha_hat[:,i-1]) - np.matmul(np.matmul(0.5*R_inv,B_h), s[:,i-1])
        u_alpha = np.reshape(u_alpha,(2,1))
        u_ast = np.asarray(eng.QCQPSolver(matlab.double(B.tolist()),matlab.double(R.tolist()),matlab.double(u_alpha.tolist()),gamma,matlab.double(P.tolist()),matlab.double(x_hat.tolist()),matlab.double(s.tolist()),i+1,n,matlab.double(start_point.tolist()), nargout = 1))
        w = np.random.normal(0,1,(n,1))
        dxdt = np.reshape(np.matmul(A, x[:,i-1]),(2,1)) + np.matmul(B,u_ast) + w
        x[:,i] = np.reshape(np.reshape(x[:,i-1],(2,1)) + dxdt*dt,2)
        v = np.random.normal(0,1,(p,1))
        v_alpha = v[pMinusS-1,:]
        attack = np.random.random((p,1))
        attack[pMinusS-1,0] = 0
        y = np.reshape(np.matmul(C,x[:,i]),(2,1)) + v + attack
        y = np.reshape(y,(2,1))
        y_alpha = np.matmul(C_alpha,x[:,i]) + v_alpha
        dPhi_alpha_dt = np.matmul(A,Phi_alpha) + np.matmul(Phi_alpha, A_h)
        dPhi_alpha_dt = dPhi_alpha_dt + Sigma_w - np.matmul(np.matmul(np.matmul(Phi_alpha,C_alpha_h)*Sigma_v_alpha,C_alpha),Phi_alpha_h)
        dPhi_dt = np.matmul(A,Phi) + np.matmul(Phi, A_h) + Sigma_w - np.matmul(np.matmul(np.matmul(np.matmul(Phi,C_h),Sigma_v_inv),C),Phi.conj().transpose())
        Phi_alpha = Phi_alpha + dt*dPhi_alpha_dt
        Theta_alpha = np.matmul(Phi_alpha,C_alpha_h)*Sigma_v_alpha
        Phi = Phi + dt*dPhi_alpha_dt
        Theta = np.matmul(np.matmul(Phi,C_h),Sigma_v_inv)
        Phi_alpha_h = Phi_alpha.conj().transpose()
        dxhat_dt = np.reshape(np.matmul(A,x_hat[:,i-1]),(2,1)) + np.matmul(B,u_ast) + np.matmul(Theta,(y - np.reshape(np.matmul(C,x_hat[:,i-1]),(2,1))))
        x_hat[:,i] = np.reshape(np.reshape(x_hat[:,i-1],(2,1)) + dt * dxhat_dt,2)
        dxhat_alpha_dt = np.reshape(np.matmul(A,x_alpha_hat[:,i-1]),(2,1)) + np.matmul(B, u_ast) + np.matmul(Theta_alpha, (y_alpha - np.matmul(C_alpha, x_alpha_hat[:,i-1])))
        x_alpha_hat[:,i] = np.reshape(np.reshape(x_alpha_hat[:,i-1],(2,1)) + dt*dxhat_alpha_dt,2)
        x_real[:,i] = np.reshape(np.reshape(x_real[:,i-1],(2,1)) + dt*(np.reshape(np.matmul(A,x_real[:,i-1]),(2,1)) + np.matmul(B,u_ast)),2)
        print(i)

if __name__ == "__main__":
    t = np.linspace(-5,0,10000);
    # t = np.reshape(t,(1,10000))
    x1 = t
    x2 = 5*np.ones(10000);
    lqgRT_v2(1, 10000, 2, 2, 2, np.array([2]), np.identity(2), np.identity(2), np.array([[1, 1], [1, -1]]), np.identity(2), np.identity(2), np.identity(2), 1e-3*np.identity(2), np.array([[-5], [5]]), 1, 1, np.array([[0], [5]]), np.array([[0], [2.5]]), t, np.vstack((x1,x2)), 5)
