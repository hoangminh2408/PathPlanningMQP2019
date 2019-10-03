#!/usr/bin/env python
import numpy as np
import rospy
import roslib
import tf
import math
from tf.transformations import euler_from_quaternion
import copy
import time
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios
from geometry_msgs.msg import PoseStamped, Twist, Pose, PoseWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf.transformations import euler_from_quaternion
#Done as a node itself

# def controller_init(robot):

# robot.vel_msg.linear.x = Xi
# robot.vel_msg.angular.z = omega
# robot.vel_pub.publish(robot.vel_msg)

print("Initializing Controller...")
T = 100;
num_steps = 2000;
n = 3;
m = 2;
p = 3;
pMinusS = np.array([2]);
A = np.identity(3);
B = np.array([[1, 0],
              [1, 0],
              [0, 1]])
C = np.identity(3);
Sigma_w = np.identity(3);
Sigma_v = np.identity(3);
Q = np.array([[1, 0, 0],
              [0, 1, 0],
              [0, 0, 1]]);
R = np.array([[1, 0],
              [0, 1]]);
start_point = np.array([0, 0, 0]);
rd_tar = 1;
rd_obs = 1;
target = np.array([2, 0.001, 0]);
obs = np.array([-1, 1]);
t = np.linspace(0.0, 100.0, num = 2000);
x1 = np.sin(t/10);
x2 = np.sin(t/20);

parametric_func = np.zeros((2,2000))
parametric_func[0] = x1
parametric_func[1] = x2

dt = T/num_steps;

s = np.zeros((2, num_steps));
stemp = np.array([[0],[0]]);
b = np.zeros((2,2,num_steps));
s[:,num_steps-1]=[0,0];
A_l = np.identity(2);
B_l = dt*np.identity(2);
Q_l = np.identity(2);
degree = 3;

g_D = rd_tar^2;
g_U = rd_obs^2;

ref_traj = parametric_func
diffrc = ref_traj[:,0]
# ref_traj[:,:] = ref_traj[:,:] - diffrc[:];
ref_length = len(ref_traj[1]);
ref_traj = np.concatenate((ref_traj, np.ones((1,ref_length))));
for i in range(0,ref_length-1):
    ref_traj[2,i] = np.arctan((ref_traj[1,i+1]-ref_traj[1,i])/(ref_traj[0,i+1]-ref_traj[0,i]));
ref_traj[2,ref_length-1] = ref_traj[2,ref_length-2];

ref_traj_dot = np.zeros((3,ref_length));
for i in range (1, ref_length):
    ref_traj_dot[0,i] = ref_traj[0,i]-ref_traj[0,i-1]
    ref_traj_dot[1,i] = ref_traj[1,i]-ref_traj[1,i-1]
    ref_traj_dot[2,i] = ref_traj[2,i]-ref_traj[2,i-1]

ref_traj_db_dot = np.zeros((3,ref_length))
for i in range(0, ref_length-1):
    ref_traj_db_dot[0,i] = ref_traj_dot[0,i+1]-ref_traj_dot[0,i]
    ref_traj_db_dot[1,i] = ref_traj_dot[1,i+1]-ref_traj_dot[1,i]
    ref_traj_db_dot[2,i] = ref_traj_dot[2,i+1]-ref_traj_dot[2,i]

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
y = np.zeros((p, num_steps));

x_p = np.zeros((n,1));
Sigma_x_p = np.zeros((n,n));

error = np.zeros((1,num_steps));
cost = np.zeros((1,num_steps));
finish = False;
# set(gcf,'CurrentCharacter','@'); % set to a dummy character
for j in range(num_steps-2, -1, -1):
    k = -(np.linalg.inv(B_l.conj().transpose()*b[:,:,j+1]*B_l+R)*B_l.conj().transpose()*b[:,:,j+1])*A_l;
    b[:,:,j] = A_l.conj().transpose()*(b[:,:,j+1]-b[:,:,j+1]*B_l*np.linalg.inv(B_l.conj().transpose()*b[:,:,j+1]*B_l+R)*B_l.conj().transpose()*b[:,:,j+1])*A_l+Q_l;
    ref_traj_a = np.array([[ref_traj[0,j+1]],[ref_traj[1,j+1]]])
    stemp = np.matmul((A_l.conj().transpose() + k.conj().transpose()*B_l.conj().transpose()),stemp) - np.matmul(Q_l,ref_traj_a); #NEED FIXING
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

u[0,0] = 2.08018939316653
u[1,0] = 1.04750514455758
# u[:,0] = ref_traj_db_dot[0:1,0]*dt - np.linalg.inv(B_l.conj().transpose()*b[:,:,0]*B_l+R)*B_l.conj().transpose()*(b[:,:,1]*A_l*x_hat[0:1,0]+s[:,0])/dt;

Xi = u[0,0]*np.cos(x_hat[2,0])*dt+u[1,0]*np.sin(x_hat[2,0])*dt
omega = dt*(u[1,0]*np.cos(x_hat[2,0])-u[0,0]*np.sin(x_hat[2,0]))/Xi

def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class pid_controller:
    def __init__(self):
        print("Creating LQR Controller Node")
        rospy.init_node('LQR_Controller')
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 2)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, callback=self.odom_callback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, callback=self.imu_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, callback=self.scan_callback)
        self.odom = Odometry()
        self.pose = Pose()
        self.twist = Twist()
        self.imu = Imu()
        self.scan = LaserScan()
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

    def lqr_loop(self, msg):
        global B, Xi, omega,n,x_hat
        for i in range(1, num_steps):
            x_temp = np.matmul(A,x_hat[:,i-1]) + np.matmul(B,np.array([[1.5*Xi*dt],[omega*dt]]));
            A_ext = np.array([[1, 0, -dt*Xi*np.sin(x_hat[2,i-1])],
                     [0, 1, dt*Xi*np.cos(x_hat[2,i-1])],
                     [0, 0, 1]])
            print(A)
            print(B)
            print(Xi)
            print(omega)
            Phi_temp = np.matmul(np.matmul(A_ext,Phi[:,:,i-1]),A_ext.conj().transpose())+Sigma_w;
            tbot_x = msg.pose.pose.position.x
            tbot_y = msg.pose.pose.position.y
            quat = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
            angles = euler_from_quaternion(quat)
            y[:,i] = [tbot_x, tbot_y, angles[0]];
            z = y[:,i]-np.matmul(C,x_temp);
            s_temp = np.matmul(np.matmul(C,Phi_temp),C.conj().transpose())+Sigma_v;
            Theta[:,:,i] = np.matmul(Phi_temp,C.conj().transpose())/s_temp;
            x_hat[:,i] = x_temp + np.matmul(Theta[:,:,i],z);
            Phi[:,:,i] = np.matmul((np.identity(n) - np.matmul(Theta[:,:,i],C)),Phi_temp)
            B = np.array([[np.cos(x_hat[2,i]),0],[np.sin(x_hat[2,i]),0],[0,1]])
            u[:,i] = ref_traj_db_dot[0:2,i]*dt -B_l.conj().transpose()/(np.matmul(np.matmul(B_l.conj().transpose(),b[:,:,i]),B_l)+R)*(np.matmul(np.matmul(b[:,:,i],A_l),x_hat[0:2,i])+s[:,i])/dt;
            Xi = u[0,i]*np.cos(x_hat[2,i])*dt+u[1,i]*np.sin(x_hat[2,i])*dt
            omega = dt*(u[1,i]*np.cos(x_hat[2,i])-u[0,i]*np.sin(x_hat[2,i]))/Xi
            twist_msg.linear.x = Xi
            twist_msg.angular.z = omega
            self.vel_pub.publish(twist_msg)

if __name__ == "__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    try:
        Robot = pid_controller()
        A = np.identity(3);
        # controller_init(Robot)
        # Robot.odom_callback(Robot.odom_msg)
        for i in range(1, num_steps):
            key = getKey()
            if key == 'e':
                Robot.vel_msg.linear.x = 0
                Robot.vel_msg.angular.z = 0
                Robot.vel_pub.publish(Robot.vel_msg)
                exit()
                break
            if Robot.odom_updated:
                # print(Robot.odom_msg)
                Robot.lqr_loop(Robot.odom_msg)
                Robot.odom_updated = False
            else:
                num_steps = num_steps - 1
        Robot.vel_msg.linear.x = 0
        Robot.vel_msg.angular.z = 0
        Robot.vel_pub.publish(Robot.vel_msg)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

