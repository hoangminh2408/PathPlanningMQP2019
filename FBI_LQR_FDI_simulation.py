#!/usr/bin/env python
import numpy as np
import rospy
import roslib
import tf
import math
from tf.transformations import euler_from_quaternion
import copy
import time
import matplotlib.pyplot as plt
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios
from geometry_msgs.msg import PoseStamped, Twist, Pose, PoseWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf.transformations import euler_from_quaternion
from pathplanningmqp.msg import transform
import cvxpy as cvx

print("Initializing Controller Variables")
print("................................")
# eng = matlab.engine.start_matlab()
T = 100;
num_steps = 5000;
tgetkey = 0;

rd_tar = 1;
rd_obs = 1;
target = np.array([2, 0.001, 0]);
obs = np.array([-1, 1]);

t = np.linspace(1, 100.0, num = num_steps);
x1 = 0.8*np.sin(t/10);
x2 = 0.8*np.sin(t/20);

parametric_func = np.zeros((2,num_steps))
parametric_func[0] = x1
parametric_func[1] = x2

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

dt = float(T)/float(num_steps);
s = np.zeros((n_l, num_steps));
b = np.zeros((n_l,n_l,num_steps));

stemp = np.array([[0],[0]]);

s[:,num_steps-1]=[0,0,0,0];
A_l = np.identity(2);
B_l = dt*np.identity(2);
Q_l = np.identity(2);
degree = 3;
B_lh = B_l.conj().transpose()
g_D = rd_tar^2;
g_U = rd_obs^2;

ref_traj = parametric_func

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

start_point = np.concatenate((start_point, ref_traj_dot[0:2, 0]));
target = ref_traj[:,ref_length-1]
ref_traj = np.concatenate((ref_traj[0:2,:], ref_traj_dot[0:2,:]))
s_coeff = np.zeros((n,degree + 1))
# for i in range(0,n):
#     s_coeff[i,:] = np.polyfit(t, ref_traj[i,:], degree)
# for i in range(0,n):
#     # ref_traj[i,:] = np.polyval(s_coeff[i,:], t)
#     break
diffrc = ref_traj[:,0]
diffrc = np.reshape(diffrc,(4,1))
ref_traj[:,:] = ref_traj[:,:] - diffrc;
start_point = ref_traj[:,0]
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
Sigma_v_alpha = np.array([[1e-6, 0, 0, 0],
                          [0, 1e-6, 0, 0],
                          [0, 0, 1e-6, 0],
                          [0, 0, 0, 1e-6]])
Sigma_v_alpha_inv = np.linalg.inv(Sigma_v_alpha)
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

P = np.zeros((n,n))
P_prev = P+0.001

while np.all(abs(P - P_prev)) >= 0.001:
    P_prev = P
    P = P + (2*Q + np.matmul(A.T, P) + np.matmul(P, A) - 0.5*np.matmul(np.matmul(np.matmul(np.matmul(P,B),R_inv),B.T),P)) * dt
BG = np.hstack((B,G))
K = np.array([[1.28718850581117,	-6.38557171480376e-16,	-2.40028202841119e-17,	0.414213562373096,	1.58212337667351e-16],
              [7.26223872225245e-17,	1.28602530882083,	0.00214337551470138,	-1.42961771886354e-15,	0.414011544555625],
              [0.414213562373095, 2.63342314537682e-16,	-5.03336638922339e-18,	0.910179721124455,	1.37263357977565e-16],
              [-1.22893379785640e-16, 0.414011544555625, 0.000690019240926037,	-1.37425909154522e-16, 0.910114698839084]])

Phi_alpha = np.zeros((n,n))
Theta_alpha = np.zeros((n,secureSensors))

Phi_alpha_prev = Phi_alpha + 0.001
Theta_alpha_prev = Theta_alpha + 0.001

while np.all(abs(Phi_alpha - Phi_alpha_prev)) >= 0.001:
    Phi_alpha_prev = Phi_alpha;
    Theta_alpha_prev = Theta_alpha;
    dPhi_alpha_dt = np.matmul(A,Phi_alpha) + np.matmul(Phi_alpha, A.T) + Sigma_w - np.matmul(np.matmul(np.matmul(np.matmul(Phi_alpha_prev,C_alpha.T),Sigma_v_alpha_inv),C_alpha),Phi_alpha.T)
    Phi_alpha = Phi_alpha + dt*dPhi_alpha_dt
    Theta_alpha = np.matmul(np.matmul(Phi_alpha,C_alpha.T),Sigma_v_alpha_inv)

P = np.zeros((n,n,num_steps));
s = np.zeros((n, num_steps));

for i in range(num_steps - 2, -1, -1):
    P[:,:,i] = P[:,:,i+1] + dt*(np.matmul(A.T,P[:,:,i+1]) + np.matmul(P[:,:,i+1],A) - 0.5*np.matmul(np.matmul(np.matmul(np.matmul(P[:,:,i+1],B),R_inv),B.T),P[:,:,i+1]) + 2*Q)
    dsdt = (A.T - 0.5*np.matmul(np.matmul(np.matmul(P[:,:,i],B),R_inv),B.T))
    dsdt = np.matmul(dsdt, s[:,i+1]) - 2*np.matmul(Q,ref_traj[:,i+1])
    s[:,i] = s[:,i+1] + dsdt*dt

Phi = np.zeros((n,n,num_steps))
Phi_alpha = np.zeros((n,n,num_steps))
Theta = np.zeros((n,p,num_steps))
Theta_alpha = np.zeros((n,secureSensors,num_steps))
gamma = 15 #CHANGE GAMMA, GAMMA CAN ONLY BE <15

for i in range(1, num_steps):
    dPhi_alpha_dt = np.matmul(A, Phi_alpha[:,:,i-1]) + np.matmul(Phi_alpha[:,:,i-1], A.T) + Sigma_w - np.matmul(np.matmul(np.matmul(np.matmul(Phi_alpha[:,:,i-1],C_alpha.T),Sigma_v_alpha_inv),C_alpha),Phi_alpha[:,:,i-1].T)
    Phi_alpha[:,:,i] = Phi_alpha[:,:,i-1] + dt*dPhi_alpha_dt
    Theta_alpha[:,:,i] = np.matmul(np.matmul(Phi_alpha[:,:,i],C_alpha.T),Sigma_v_alpha_inv)

    dPhi_dt = np.matmul(A,Phi[:,:,i-1]) + np.matmul(Phi[:,:,i-1], A.T) + Sigma_w - np.matmul(np.matmul(np.matmul(np.matmul(Phi[:,:,i-1],C.T),Sigma_v_inv),C),Phi[:,:,i-1].T)
    Phi[:,:,i] = Phi[:,:,i-1] + dt*dPhi_dt
    Theta[:,:,i] = np.matmul(np.matmul(Phi[:,:,i],C.T),Sigma_v_inv)

first_step_angle = np.arctan((ref_traj[1,1] - ref_traj[1,0])/(ref_traj[0,1] - ref_traj[0,0]));
init_angle = 0;
theta = first_step_angle-init_angle;
state_init = [0,0,1e-4];
B_ind = 0;
Xi = np.zeros((1,num_steps))
omega = np.zeros((1,num_steps))
angles = 0
# DONE SETTING UP Variables
def plotting():
    global ref_traj, y, dt, T, num_steps, elapsed_time
    plt.ioff()
    fig1 = plt.figure()
    fig1.suptitle("Reference trajectory vs Actual trajectory\n " + "dt = " + str(dt) + "; T = " + str(T) + "; num_steps = " + str(num_steps) + "; Elapsed time: " + str(elapsed_time))
    plt.plot(ref_traj[0,:], ref_traj[1,:], label = 'Reference trajectory')
    plt.plot(y[0,:], y[1,:], label = 'Actual trajectory')

    fig2 = plt.figure()
    fig2.suptitle("Mean Square Error\n" + "dt = " + str(dt) + "; T = " + str(T) + "; num_steps = " + str(num_steps) + "; Elapsed time: " + str(elapsed_time))
    error = np.zeros(num_steps)
    for i in range(0, num_steps):
        error[i] = math.pow((np.linalg.norm(x_hat[0:2,i]-ref_traj[0:2,i])),2)/2;
    plt.plot(error)

    fig3 = plt.figure()
    fig3.suptitle("Reference x vs Actual x")
    plt.plot(ref_traj[0,:], label = "Reference x")
    plt.plot(y[0,:], label = "Actual x")

    fig4 = plt.figure()
    fig4.suptitle("Reference y vs Actual y")
    plt.plot(ref_traj[1,:], label = "Reference y")
    plt.plot(y[1,:], label = "Actual y")

    fig5 = plt.figure()
    fig5.suptitle("Reference y vs Lidar's y")
    plt.plot(ref_traj[1,:], label = "Reference y")
    plt.plot(y[2,:], label = "Lidar's y")
    plt.show()

class lqr_controller:
    def __init__(self):
        print("Creating LQR Controller Node")
        print("............................")
        rospy.init_node('LQR_Controller')
        self.listener = tf.TransformListener()

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 2)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, callback=self.odom_callback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, callback=self.imu_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, callback=self.scan_callback)
        self.trans_sub = rospy.Subscriber('/linear_trans', transform, callback=self.trans_callback)
        self.odom_msg = Odometry()
        self.pose_msg = Pose()
        self.vel_msg = Twist()
        self.imu_msg = Imu()
        self.scan_msg = LaserScan()
        self.trans_msg = transform()
        self.odom_updated = False
        self.imu_updated = False
        self.scan_updated = False
        self.trans_updated = False

    def odom_callback(self, msg):
        self.odom_msg = msg
        self.odom_updated = True

    def imu_callback(self, msg):
        self.imu_msg = msg
        self.imu_updated = True

    def scan_callback(self, msg):
        self.scan_msg = msg
        self.scan_updated = True
    def trans_callback(self, msg):
        self.trans_msg = msg
        self.trans_updated = True

    def lqr_loop(self, msg, i, trans_msg):
        global A, B, Xi, omega,n,x_hat,dt,angles
        if i == 0:
            stime1 = time.time()
            tbot_x = msg.pose.pose.position.x
            tbot_y = msg.pose.pose.position.y
            quat = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
            angles = euler_from_quaternion(quat)
            print(tbot_x)
            print(tbot_y)

            y_lidar = trans_msg.linear_transform#from transform listener
            print("Y LIDAR IS:" + str(y_lidar))
            y[:,0] = [tbot_x, tbot_y, y_lidar, 0, 0]
            y_alpha[:,0] = [y[0,0],y[1,0],y[3,0],y[4,0]]
            u[:,0] = -0.5*(np.matmul(np.matmul(np.matmul(R_inv,B.T),P[:,:,0]),x_hat[:,0])) - 0.5*(np.matmul(np.matmul(R_inv,B.T),s[:,0]))
            u_ast[:,0] = u[:,0]
            Xi[0,0] = 0.9*T*dt*(u_ast[0,0]*np.cos(angles[2]) + u_ast[1,0]*np.sin(angles[2]))
            if Xi[0,i] != 0:
                omega[0,0] = 0.9*T*dt*(u_ast[1,0]*np.cos(angles[2]) - u_ast[0,0]*np.sin(angles[2]))/Xi[0,0]
            else:
                omega[0,0] = 0
            self.vel_msg.linear.x = Xi[0,0]
            self.vel_msg.angular.z = omega[0,0]
            self.vel_pub.publish(self.vel_msg)
            elapsed = time.time() - stime1
            while elapsed < dt:
                elapsed = time.time() - stime1
        else:
            stime1 = time.time()
            print("Step number " + str(i))

            dxhat_dt = np.reshape(np.matmul(A,x_hat[:,i-1]),(4,1)) + np.reshape(np.matmul(B,u_ast[:,i-1]),(4,1)) + np.matmul(Theta[:,:,i-1],np.reshape(y[:,i-1],(5,1)) - np.reshape(np.matmul(C,x_hat[:,i-1]),(5,1)))
            print("DXHAT_DT: " + str(dxhat_dt))
            x_hat[:,i] = np.reshape(np.reshape(x_hat[:,i-1],(4,1)) + dt * dxhat_dt, 4)
            print("X_HAT: " + str(x_hat[:,i]))
            dxhat_alpha_dt = np.reshape(np.matmul(A,x_alpha_hat[:,i-1]),(4,1)) + np.reshape(np.matmul(B, u_ast[:,i-1]),(4,1)) + np.matmul(Theta_alpha[:,:,i-1], (np.reshape(y_alpha[:,i-1],(4,1)) - np.reshape(np.matmul(C_alpha, x_alpha_hat[:,i-1]),(4,1))))
            print("DXHAT_ALPHA_DT: " + str(dxhat_dt))
            x_alpha_hat[:,i] = np.reshape(np.reshape(x_alpha_hat[:,i-1],(4,1)) + dt*dxhat_alpha_dt,4)
            print("X_ALPHA_HAT: " + str(x_alpha_hat[:,i]))
            u[:,i] = -0.5*(np.matmul(np.matmul(np.matmul(R_inv,B.T),P[:,:,i]),x_alpha_hat[:,i])) - 0.5*(np.matmul(np.matmul(R_inv,B.T),s[:,i]))
            print("U: " + str(u[:,i]))
            z = cvx.Variable(2)
            obj = cvx.Minimize(cvx.quad_form(z,R) + np.matmul(B.T,np.reshape(np.matmul(x_hat[:,i], P[:,:,i]) + s[:,i],(4,1))).T*z)
            prob = cvx.Problem(obj,[0.5*cvx.quad_form(z,np.eye(2))+(-2*u[:,i].T*z) + (np.matmul(u[:,i].T,u[:,i])) - math.pow(gamma,2) <= 0])
            prob.solve(solver = 'SCS')
            print("RESULTS: " + str(z.value))
            u_ast[:,i] = np.reshape(2*z.value,2)
            print("Control input: " + str(u_ast[:,i]))
            Xi[0,i] = 0.9*T*dt*(u_ast[0,i]*np.cos(angles[2]) + u_ast[1,i]*np.sin(angles[2]))
            if Xi[0,i] != 0:
                omega[0,i] = 0.9*T*dt*(u_ast[1,i]*np.cos(angles[2]) - u_ast[0,i]*np.sin(angles[2]))/Xi[0,i]
            else:
                omega[0,i] = 0
            print("Xi is: " + str(Xi[0,i]))
            print("omega is: " + str(omega[0,i]))
            self.vel_msg.linear.x = Xi[0,i]
            self.vel_msg.angular.z = omega[0,i]
            self.vel_pub.publish(self.vel_msg)
            elapsed = time.time() - stime1
            while elapsed < dt:
                elapsed = time.time() - stime1
            print("Elapsed time:" + str(elapsed))

            tbot_x = msg.pose.pose.position.x
            tbot_y = msg.pose.pose.position.y
            quat = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
            angles = euler_from_quaternion(quat)
            print("angles: " + str(angles))

            y_lidar = trans_msg.linear_transform
            print("Y LIDAR IS:" + str(y_lidar))
            print("ACTUAL Y IS: " + str(tbot_y))
            print("----------------------")
            a = 0.01*np.random.randn()
            y[0,i] = tbot_x
            y[1,i] = tbot_y
            y[2,i] = y_lidar+a
            y[3,i] = (tbot_x - y[0,i-1])/dt
            y[4,i] = (tbot_y - y[1,i-1])/dt
            y_alpha[:,i] = [y[0,i],y[1,i],y[3,i],y[4,i]]

if __name__ == "__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    try:
        start_time = time.time()
        Robot = lqr_controller()
        for i in range(0, num_steps):
            Robot.lqr_loop(Robot.odom_msg,i,Robot.trans_msg)
        Robot.vel_msg.linear.x = 0
        Robot.vel_msg.angular.z = 0
        Robot.vel_pub.publish(Robot.vel_msg)
        elapsed_time = time.time() - start_time
        print("x_hat is: " + str(x_hat))
        print("u is: " + str(u))
        plotting()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

