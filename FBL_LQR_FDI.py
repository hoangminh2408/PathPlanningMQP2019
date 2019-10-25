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
import time
import sys, select, os
import random
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios
from geometry_msgs.msg import PoseStamped, Twist, Pose, PoseWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf.transformations import euler_from_quaternion

print("Initializing Controller Variables")
print("................................")
T = 100;
num_steps = 50000;
tgetkey = 0;
random.seed(1)

n = 3;
m = 2;
p = 3;
pMinusS = np.array([2]);
A = np.identity(3);
B = np.array([[1, 0],
              [1, 0],
              [0, 1]])
C = np.identity(3);
Sigma_w = np.array([[1e-6, 0, 0],
                    [0, 1e-6, 0],
                    [0, 0, 1e-6]]);
Sigma_v = np.array([[1e-6, 0, 0],
                    [0, 1e-6, 0],
                    [0, 0, 1e-6]]);
Q = np.array([[1, 0, 0],
              [0, 1, 0],
              [0, 0, 1]]);
R = np.array([[1, 0],
              [0, 1]]);

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
s = np.zeros((2, num_steps));
stemp = np.array([[0],[0]]);
b = np.zeros((2,2,num_steps));
s[:,num_steps-1]=[0,0];
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

ref_length = len(ref_traj[2]);
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
start_point = ref_traj[:,0];
target = ref_traj[:,ref_length-1]

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
x0 = start_point;
x_hat[:,0] = x0;
x_alpha_hat[:,0] = x0;
x_real[:,0] = x0;

G = Sigma_w;

P = np.zeros((n,n,num_steps));

Sigma_x = np.zeros((n, n, num_steps));

u = np.zeros((m, num_steps));
real_y = np.zeros((p,num_steps));
y = np.zeros((p, num_steps));

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

P = np.zeros(b.shape)
for i in range (0,num_steps):
    P[:,:,i] = np.subtract(b[:,:,i],Q_l);

Phi = np.zeros((n,n,num_steps));
Theta = np.zeros((n,p,num_steps));
Theta[:,:,0] = Phi[:,:,0]*C.conj().transpose()*Sigma_v_inv;

uu1 = np.linalg.inv(np.matmul(np.matmul(B_lh,b[:,:,0]),B_l)+R)
uu2 = np.matmul(uu1,B_lh)
uu3 = (np.matmul(np.matmul(b[:,:,0],A_l),x_hat[0:2,0])+s[:,0])
uu3 = np.reshape(uu3,(2,1))
uu4 = np.matmul(uu2,uu3)/dt
uu4 = np.reshape(uu4,(2,1))
uu5 = np.reshape(ref_traj_db_dot[0:2,0]*dt,(2,1)) - uu4

u[:,0] = np.reshape(uu5,(1,2))

start_time = 0
elapsed_time = 0
def plotting():
    global ref_traj, real_y, y, dt, T, num_steps, elapsed_time
    plt.ioff()
    fig1 = plt.figure()
    fig1.suptitle("Reference trajectory vs Actual trajectory\n " + "dt = " + str(dt) + "; T = " + str(T) + "; num_steps = " + str(num_steps) + "; Elapsed time: " + str(elapsed_time))
    plt.plot(ref_traj[0,:], ref_traj[1,:], label = 'Reference trajectory')
    plt.plot(real_y[0,:], real_y[1,:], label = 'Actual trajectory')

    fig2 = plt.figure()
    fig2.suptitle("Mean Square Error\n" + "dt = " + str(dt) + "; T = " + str(T) + "; num_steps = " + str(num_steps) + "; Elapsed time: " + str(elapsed_time))
    error = np.zeros(num_steps)
    for i in range(0, num_steps):
        error[i] = math.pow((np.linalg.norm(x_hat[0:2,i]-ref_traj[0:2,i])),2)/2;
    plt.plot(error)

    fig3 = plt.figure()
    fig3.suptitle("Reference x vs Actual x")
    plt.plot(ref_traj[0,:], label = "Reference x")
    plt.plot(real_y[0,:], label = "Actual x")

    fig4 = plt.figure()
    fig4.suptitle("Reference y vs Actual y")
    plt.plot(ref_traj[1,:], label = "Reference y")
    plt.plot(real_y[1,:], label = "Actual y")

    plt.show()

def getKey():
    global tgetkey
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], tgetkey)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class lqr_controller:
    def __init__(self):
        print("Creating LQR Controller Node")
        print("............................")
        rospy.init_node('LQR_Controller')
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 2)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, callback=self.odom_callback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, callback=self.imu_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, callback=self.scan_callback)
        self.odom_msg = Odometry()
        self.pose_msg = Pose()
        self.vel_msg = Twist()
        self.imu_msg = Imu()
        self.scan_msg = LaserScan()
        self.odom_updated = False
        self.imu_updated = False
        self.scan_updated = False

    def odom_callback(self, msg):
        self.odom_msg = msg
        self.odom_updated = True

    def imu_callback(self, msg):
        self.imu_msg = msg
        self.imu_updated = True

    def scan_callback(self, msg):
        self.scan_msg = msg
        self.scan_updated = True

    def lqr_loop(self, msg, i):
        global A, B, Xi, omega,n,x_hat,dt
        # try using ros timer to control the time Step , x and y vs time, mean square error for trajectory
        if i == 0:
            stime1 = time.time()
            Xi = u[0,0]*np.cos(x_hat[2,0])*dt+u[1,0]*np.sin(x_hat[2,0])*dt
            omega = dt*(u[1,0]*np.cos(x_hat[2,0])-u[0,0]*np.sin(x_hat[2,0]))/Xi
            self.vel_msg.linear.x = Xi
            self.vel_msg.angular.z = omega
            self.vel_pub.publish(self.vel_msg)
            elapsed = time.time() - stime1
            while elapsed < dt:
                elapsed = time.time() - stime1
        else:
            stime1 = time.time()
            print("Step number " + str(i))
            x_temp = np.matmul(x_hat[:,i-1],A).reshape(-1, 1) + np.matmul(B,np.array([[1.5*Xi*dt],[omega*dt]]));
            A_ext = np.array([[1, 0, -dt*Xi*np.sin(x_hat[2,i-1])],
                     [0, 1, dt*Xi*np.cos(x_hat[2,i-1])],
                     [0, 0, 1]])
            Phi_temp = np.matmul(np.matmul(A_ext,Phi[:,:,i-1]),A_ext.conj().transpose())+Sigma_w;
            tbot_real_x = msg.pose.pose.position.x
            tbot_real_y = msg.pose.pose.position.y
            tbot_x = tbot_real_x
            tbot_y = tbot_real_y
            # FDI VERSION 1: small injection every step
            # randnumber = random.uniform(-0.500, 0.500)
            # tbot_x = tbot_x
            # tbot_y = tbot_y + randnumber
            # print("Random number: " + str(randnumber))
            # FDI VERSION 2: (possibly) big injection every 500 steps
            #
            if i % 20 == 0:
                randnumber1 = random.uniform(-1.5,1.5)
                randnumber2 = random.uniform(-1.5,1.5)
                tbot_x = tbot_x + randnumber1
                tbot_y = tbot_y + randnumber2
                print("Random number: " + str(randnumber1) + str(randnumber2))
            quat = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
            angles = euler_from_quaternion(quat)
            real_y[:,i] = [tbot_real_x, tbot_real_y, angles[2]];
            y[:,i] = [tbot_x, tbot_y, angles[2]]; #add random value to y
            print("TBOT position is : X = " + str(tbot_x) + " Y = " + str(tbot_y))
            z = y[:,i].reshape(-1,1)-np.matmul(C,x_temp);
            s_temp = np.matmul(np.matmul(C,Phi_temp),C.conj().transpose())+Sigma_v;
            Theta[:,:,i] = np.matmul(np.matmul(Phi_temp,C.conj().transpose()),np.linalg.inv(s_temp));
            x_hat[:,i] = np.reshape(x_temp + np.matmul(Theta[:,:,i],z),3);
            Phi[:,:,i] = np.matmul((np.identity(n) - np.matmul(Theta[:,:,i],C)),Phi_temp)
            B = np.array([[np.cos(x_hat[2,i]),0],[np.sin(x_hat[2,i]),0],[0,1]])
            uu1 = np.linalg.inv(np.matmul(np.matmul(B_lh,b[:,:,i]),B_l)+R)
            uu2 = np.matmul(uu1,B_lh)
            uu3 = (np.matmul(np.matmul(b[:,:,i],A_l),x_hat[0:2,i])+s[:,i])
            uu3 = np.reshape(uu3,(2,1))
            uu4 = np.matmul(uu2,uu3)/dt
            uu4 = np.reshape(uu4,(2,1))
            uu5 = np.reshape(ref_traj_db_dot[0:2,i]*dt,(2,1)) - uu4
            u[:,i] = np.reshape(uu5,(1,2))
            Xi = u[0,i]*np.cos(x_hat[2,i])*dt+u[1,i]*np.sin(x_hat[2,i])*dt
            if Xi != 0:
                omega = dt*(u[1,i]*np.cos(x_hat[2,i])-u[0,i]*np.sin(x_hat[2,i]))/Xi
            else:
                omega = 0
            print("Xi is: " + str(Xi))
            print("omega is: " + str(omega))
            self.vel_msg.linear.x = Xi
            self.vel_msg.angular.z = omega
            self.vel_pub.publish(self.vel_msg)
            elapsed = time.time() - stime1
            while elapsed < dt:
                elapsed = time.time() - stime1
            print("Elapsed time:" + str(elapsed))
            print("----------------------")
if __name__ == "__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    try:
        Robot = lqr_controller()
        start_time = time.time()
        for i in range(0, num_steps):
            key = getKey()
            if key == 'e':
                Robot.vel_msg.linear.x = 0
                Robot.vel_msg.angular.z = 0
                Robot.vel_pub.publish(Robot.vel_msg)
                exit()
                break
            Robot.lqr_loop(Robot.odom_msg,i)
        Robot.vel_msg.linear.x = 0
        Robot.vel_msg.angular.z = 0
        Robot.vel_pub.publish(Robot.vel_msg)
        elapsed_time = time.time() - start_time
        plotting()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

