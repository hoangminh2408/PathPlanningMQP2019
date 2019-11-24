#!/usr/bin/env python
import numpy as np
# import rospy
# import roslib
# import tf
import math
# from tf.transformations import euler_from_quaternion
import copy
import time
# import matplotlib.pyplot as plt
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios
# from geometry_msgs.msg import PoseStamped, Twist, Pose, PoseWithCovariance
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import Imu, LaserScan
# from tf.transformations import euler_from_quaternion

print("Initializing Controller Variables")
print("................................")

T = 50;
num_steps = 50000;
tgetkey = 0;

n = 4;
n_l = 4;
m = 2;
p = 5;
pMinusS = np.array([1, 2, 4, 5]);
A = np.array([[0, 0, 1, 0],
              [0, 0, 0, 1],
              [0, 0, 0, 0],
              [0 ,0 ,0 ,0]]);
B = np.array([[0, 0],
              [0, 0],
              [1, 0],
              [0, 1]])
C = np.array([[1, 0, 0, 0],
              [0, 1, 0, 0],
              [0, 1, 0, 0],
              [0, 0 ,1, 0],
              [0, 0 ,0 ,1]]);

Sigma_w = np.array([[1e-6, 0, 0, 0],
                    [0, 1e-6, 0, 0],
                    [0, 0, 1e-6, 0],
                    [0, 0, 0, 1e-6]]);
Sigma_v = np.array([[1e-6, 0, 0, 0 ,0],
                    [0, 1e-6, 0, 0, 0],
                    [0, 0, 6*1e-4, 0, 0],
                    [0, 0, 0, 1e-6, 0],
                    [0, 0, 0, 0, 1e-6]]);
Q = np.identity(4);
R = np.identity(2);

rd_tar = 1;
rd_obs = 1;
target = np.array([2, 0.001, 0]);
obs = np.array([-1, 1]);
t = np.linspace(0.0, 100.0, num = num_steps);
x1 = 0.8*np.sin(t/10);
x2 = 0.8*np.sin(t/20);

parametric_func = np.zeros((2,num_steps))
parametric_func[0] = x1
parametric_func[1] = x2

dt = float(T)/float(num_steps);
s = np.zeros((n_l, num_steps));
stemp = np.array([[0],[0]]);
b = np.zeros((n_l,n_l,num_steps));
s[:,num_steps-1]=[0,0,0,0];
A_l = np.identity(2);
B_l = dt*np.identity(2);
Q_l = np.identity(2);
degree = 3;
B_lh = B_l.conj().transpose()
g_D = rd_tar^2;
g_U = rd_obs^2;

ref_traj = parametric_func
diffrc = ref_traj[:,0]

ref_length = len(ref_traj[1]);
ref_traj = np.concatenate((ref_traj, np.ones((1,ref_length))));

for i in range(0,ref_length-1):
    ref_traj[2,i] = np.arctan((ref_traj[1,i+1]-ref_traj[1,i])/(ref_traj[0,i+1]-ref_traj[0,i]));
ref_traj[2,ref_length-1] = ref_traj[2,ref_length-2];
start_point = ref_traj[:,0]

ref_traj_dot = np.zeros((3,ref_length));
for i in range (1, ref_length):
    ref_traj_dot[0,i] = (ref_traj[0,i]-ref_traj[0,i-1])/dt
    ref_traj_dot[1,i] = (ref_traj[1,i]-ref_traj[1,i-1])/dt
    ref_traj_dot[2,i] = (ref_traj[2,i]-ref_traj[2,i-1])/dt

ref_traj_db_dot = np.zeros((3,ref_length))
for i in range(0, ref_length-1):
    ref_traj_db_dot[0,i] = (ref_traj_dot[0,i+1]-ref_traj_dot[0,i])/dt
    ref_traj_db_dot[1,i] = (ref_traj_dot[1,i+1]-ref_traj_dot[1,i])/dt
    ref_traj_db_dot[2,i] = (ref_traj_dot[2,i+1]-ref_traj_dot[2,i])/dt

rd = np.zeros((2,ref_length-1));
for i in range (0, ref_length-1):
    rd[0,i] = ref_traj[0,i+1]-ref_traj[0,i];
    rd[1,i] = ref_traj[1,i+1]-ref_traj[1,i];

rdd = np.zeros((2,ref_length-2));
for i in range(0,ref_length-2):
    rdd[0,i] = rd[0,i+1]-rd[0,i];
    rdd[1,i] = rd[1,i+1]-rd[1,i];

# redefine start point and target
# start_point = ref_traj(:,1);
start_point = np.concatenate((start_point, ref_traj_dot[0:1, 0]));
target = ref_traj[:,ref_length-1]

s_coeff = np.zeros((n,degree + 1))
for i in range(0,n):
    s_coeff[i,:] = np.polyfit(t, np.reshape(ref_traj[i,:],num_steps), degree)
for i in range(0,n):
    ref_traj[i,:] = np.polyval(s_coeff[i,:], t)

if dt <= 0:
    dt = 1e-4;

if n <= 0:
    n = 2;

if m <= 0:
    m = 2;

if p <= 0:
    p = 2;

[secureSensors] = pMinusS.shape;
if secureSensors > p:
    print('The number of secure sensors should be smaller than or equal to the total number of sensors.')

[rowA, colA] = A.shape;
if rowA != n or colA != n:
    print('A should be an n*n matrix.')

[rowB, colB] = B.shape;
if rowB != n or colB != m:
    print('B should be an n*m matrix.')

[rowC, colC] = C.shape;
if rowC != p or colC != n:
    print('C should be an p*n matrix.')

[rowQ, colQ] = Q.shape;
if rowQ != n or colQ != n:
    print('Q should be an n*n matrix.')

[rowR, colR] = R.shape;
if rowR != m or colR != m:
    print('R should be an m*m matrix.')

C_alpha = C[pMinusS-1,:];
Sigma_v_alpha = Sigma_v[pMinusS-1, pMinusS-1];
R_inv = np.linalg.inv(R);
Sigma_v_inv = np.linalg.inv(Sigma_v);

x_hat = np.zeros((n, num_steps));
x_alpha_hat = np.zeros((n,num_steps));
x_real = np.zeros((n,num_steps));
x0 = ref_traj[:,0];
x_hat[:,0] = x0;
x_alpha_hat[:,0] = x0;
x_real[:,0] = x0;

G = np.identity(n);

P = np.zeros((n,n,num_steps));

Sigma_x = np.zeros((n, n, num_steps));

u = np.zeros((m, num_steps));
u_ast = np.zeros((m, num_steps))
u_diff = np.zeros((m, num_steps))

y = np.zeros((p, num_steps));
y_alpha = np.zeros((secureSensors, num_steps));
y_dist = np.zeros((1, num_steps));

x_p = np.zeros((n,1));
Sigma_x_p = np.zeros((n,n));

error = np.zeros((1,num_steps));
cost = np.zeros((1,num_steps));
finish = False;

for j in range(num_steps-2, -1, -1):
    k = -(np.linalg.inv(B_l.conj().transpose()*b[:,:,j+1]*B_l+R)*B_l.conj().transpose()*b[:,:,j+1])*A_l;
    b[:,:,j] = A_l.conj().transpose()*(b[:,:,j+1]-b[:,:,j+1]*B_l*np.linalg.inv(B_l.conj().transpose()*b[:,:,j+1]*B_l+R)*B_l.conj().transpose()*b[:,:,j+1])*A_l+Q_l;
    ref_traj_a = np.array([[ref_traj[0,j+1]],[ref_traj[1,j+1]]])
    stemp = np.matmul((A_l.conj().transpose() + k.conj().transpose()*B_l.conj().transpose()),stemp) - np.matmul(Q_l,ref_traj_a);
    s[0,j] = stemp[0]
    s[1,j] = stemp[1]

first_step_angle = np.arctan((ref_traj[1,1] - ref_traj[1,0])/(ref_traj[0,1] - ref_traj[0,0]));
init_angle = 0;
theta = first_step_angle-init_angle;
state_init = [0,0,1e-4];
B_ind = 0;

B = np.array([[np.cos(0.0079),0],
              [np.sin(0.0079),0],
              [0,1]]);
y[:,0] = state_init;

P = np.zeros((n,n))
P_prev = P+0.001

while np.all(abs(P - P_prev)) >= 0.001:
    P_prev = P
    P = P + (2*Q + np.matmul(A.T, P) + np.matmul(P, A) - 0.5*np.matmul(np.matmul(np.matmul(np.matmul(P,B),R_inv),B.T),P)) * dt

for i in range (0,num_steps):
    P[:,:,i] = np.subtract(b[:,:,i],Q_l);

K = np.array([[22.8662692463450,22.8662692463450],[22.8662692463450,22.8662692463450]])

Phi_alpha = np.zeros((n,n))
Theta_alpha = np.zeros((n,secureSensors))

Phi_alpha_prev = Phi_alpha + 0.001
Theta_alpha_prev = Theta_alpha + 0.001

while np.all(abs(Phi_alpha - Phi_alpha_prev)) >= 0.001:
    Phi_alpha_prev = Phi_alpha;
    Theta_alpha_prev = Theta_alpha;
    dPhi_alpha_dt = np.matmul(A,Phi_alpha) + np.matmul(Phi_alpha, A.T) + Sigma_w - np.matmul(np.matmul(np.matmul(np.matmul(Phi_alpha_prev,C_alpha.T),np.linalg.inv(Sigma_v_alpha)),C_alpha),Phi_alpha.T)
    Phi_alpha = Phi_alpha + dt*dPhi_alpha_dt
    Theta_alpha = np.matmul(np.matmul(Phi_alpha,C_alpha.T),np.linalg.inv(Sigma_v_alpha))
Phi = np.zeros((n,n,num_steps));
Theta = np.zeros((n,p,num_steps));
Theta[:,:,0] = Phi[:,:,0]*C.conj().transpose()*Sigma_v_inv;
