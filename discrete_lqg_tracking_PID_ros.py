import numpy as np
#import rospy
T = 100;
num_steps = 2000;
n = 3;
m = 2;
p = 3;
pMinusS = [2];
A = np.identity(3);
B = [[1, 0],
     [1, 0],
     [0, 1]]
C = np.identity(3);
Sigma_w = np.identity(3);
Sigma_v = np.identity(3);
Q = [[1, 0, 0],
     [0, 1, 0],
     [0, 0, 1]];
R = [[1, 0],
     [0, 1]];
start_point = [0, 0, 0];
rd_tar = 1;
rd_obs = 1;
target = [2, 0.001, 0];
obs = [-1, 1];
t = np.linspace(0, 99, num = 100);
kp1 = 1;
kp2 = 1;
kd1 = 0.8;
kd2 = 0.8;
x1 = np.sin(t/10);
x2 = np.sin(t/20);
parametric_func = [x1, x2];
dt = T/num_steps;
s = np.zeros((n, num_steps));
b = np.zeros((n,n,num_steps));
s[:,num_steps-1]=[0;0];
b[:,:,num_steps-1]=np.zeros(3);
degree = 3;
dt = T/num_steps;

#for i in range(1,n-1):
#    eval(print('syms x %d', i))

g_D = rd_tar^2;
g_U = rd_obs^2;

# for i in range(1, n-1):
#     eval(sprintf('g_D = g_D - (x%d - target(i))^2', i));
#     eval(sprintf('g_U = g_U - (x%d - obs(i))^2', i));
# end

ref_traj = parametric_func
