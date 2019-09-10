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
Sigma_w = [[1e-6, 0, 0],
           [0, 1e-6, 0],
           [0, 0, 1e-6]];
Sigma_v = [[1e-6, 0, 0],
           [0, 1e-6, 0],
           [0, 0, 1e-6]];
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
t = np.linspace(0.0, 100.0, num = 2000);
x1 = np.sin(t/10);
x2 = np.sin(t/20);
parametric_func = [x1, x2];
dt = T/num_steps;
s = np.zeros((2, num_steps));
b = np.zeros((2,2,num_steps));
#s(:,num_steps)=[0;0];
A_l = np.identity(2);
B_l = dt*np.identity(2);
Q_l = np.identity(2);
degree = 3;
