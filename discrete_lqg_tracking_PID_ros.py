import numpy as np
#import rospy
T = 100;
num_steps = 100;
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
t = np.linspace(0, 99, num = 100);

kp1 = 1;
kp2 = 1;
kd1 = 0.8;
kd2 = 0.8;

x1 = np.sin(t/10);
x2 = np.sin(t/20);

parametric_func = np.zeros((2,100))
parametric_func[0] = x1
parametric_func[1] = x2

dt = T/num_steps;
s = np.zeros((n, num_steps));
b = np.zeros((n,n,num_steps));

s[:,num_steps-1]=[0,0,0];
b[:,:,num_steps-1]=np.zeros(3);
degree = 3;
dt = T/num_steps;

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
    ref_traj_dot[0,i] = ref_traj[0,i]-ref_traj[0,i-1];
    ref_traj_dot[1,i] = ref_traj[1,i]-ref_traj[1,i-1];
    ref_traj_dot[2,i] = ref_traj[2,i]-ref_traj[2,i-1];

ref_traj_db_dot = np.zeros((3,ref_length));
for i in range(0, ref_length-1):
    ref_traj_db_dot[0,i] = ref_traj_dot[0,i+1]-ref_traj_dot[0,i];
    ref_traj_db_dot[1,i] = ref_traj_dot[1,i+1]-ref_traj_dot[1,i];
    ref_traj_db_dot[2,i] = ref_traj_dot[2,i+1]-ref_traj_dot[2,i];
# add the third dimension: xdd, ydd
# rd = [x_dot; y_dot];
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

#parameters initialization
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
# ---------ROS STUFF-------------
# robot = rospublisher('/cmd_vel');
# laser = rossubscriber('/scan');
# imu = rossubscriber('/imu');
# odom = rossubscriber('/odom');
# stmsg = rosmessage(odom);
# msg = rosmessage(robot);
#---------ROS STUFF-------------

finish = False;
# set(gcf,'CurrentCharacter','@'); % set to a dummy character

first_step_angle = np.arctan((ref_traj[1,1] - ref_traj[1,0])/(ref_traj[0,1] - ref_traj[0,0]));
init_angle = 0;
theta = first_step_angle-init_angle;
state_init = [0,0,1e-4];
#Initial B
B_ind = 0;

B = np.array([[np.cos(0.0079),0],
              [np.sin(0.0079),0],
              [0,1]]);

y[:,0] = state_init;

# plot(y(1,1),y(2,1))
# hold on
for i in range (0,num_steps):
    P[:,:,i] = np.subtract(b[:,:,i],Q);

Phi = np.zeros((n,n,num_steps));
Theta = np.zeros((n,p,num_steps));
Theta[:,:,0] = Phi[:,:,0]*C.conj().transpose()*Sigma_v_inv;
for i in range (1,num_steps):
    Phi_temp = A*Phi[:,:,i-1]*A.conj().transpose()+Sigma_w;
    Theta[:,:,i] = Phi_temp*C.conj().transpose()*np.linalg.inv(C*Phi_temp*C.conj().transpose()+Sigma_v);
    Phi[:,:,i] = A*Phi_temp*A.conj().transpose() + Sigma_w - Phi_temp*C.conj().transpose()*np.linalg.inv(C*Phi_temp*C.conj().transpose()+Sigma_v)*C*Phi_temp.conj().transpose();

u[0,0] = ref_traj_db_dot[0,0] + kp1*(ref_traj[0,0]-y[0,0]);
u[1,0] = ref_traj_db_dot[1,0] + kp2*(ref_traj[1,0]-y[1,0]);
Xi = u[0,0]*np.cos(y[2,0])*dt+u[1,0]*np.sin(y[2,0])*dt
omega = (u[1,0]*np.cos(y[2,0])-u[0,0]*np.sin(y[2,0]))/Xi
if omega > 0.3:
    omega = 0.3
elif omega < -0.3:
    omega = -0.3
print(Xi)
print(omega)
# msg.Angular.Z = omega;
# msg.Linear.X = Xi;
# send(robot,msg);
