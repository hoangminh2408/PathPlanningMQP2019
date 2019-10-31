LQG Reference Tracking

lqgRT is a MATLAB toolbox for solving linear quadratic Guassian (LQG) tracking program following given reference trajectory under a user-specific system. 
More details can be found in the paper: LQG Reference Tracking with Safety and Reachability Guarantees under False Data Injection Attacks, Luyao Niu, Zhouchi Li, and Andrew Clark

The definitions of the inputs are as follows:

T                     -- the final time of the system
num_steps             -- the number of iterations during T, 1000 is not enoughœ
n                     -- the dimension of the system state
m                     -- the dimension of the system input
p                     -- the dimension of the system observation
pMinusS               -- the index list of the secure sensors(row vector)
A                     -- n*n state matrix
B                     -- n*m input matrix
C                     -- p*n output matrix
Sigma_w               -- n*n autocorrelation matrix
Sigma_v               -- p*p autocorrelation matrix
Q                     -- n*n cost matrix
R                     -- m*m cost matrix
start_point           -- n*1 vector, the initial state of the system
rd_tar                -- the radius of the target area
rd_obs                -- the radius of the obstacle area
target                -- n*1 vector, the center of the target area
obstacle              -- n*1 vector, the center of the obstacle area
t                     -- 1*num_steps vector, the parameter of the parametric equations of the reference trajectory
parametric_func       -- n polynomial parametric equations with parameter t
degree                -- the degree of the polyfit for the reference trajectory


We provide three versions of lqgRT function, corresponding to three different methods to enter inputs. They are:
lqgRT.m
Users can enter the inputs in the command window interactively. One example of the inputs is as follow:
Final time of the system: 1
Number of iterations during T: 1e4
Dimension of the system state: 2
Dimension of the system input: 2
Dimension of the system observation: 2
Index list of the secure sensors(row vector): [2]
nxn state matrix: eye(2)
nxm input matrix: eye(2)
pxn output matrix: [1 1; 1 -1]
nxn autocorrelation matrix: eye(2)
pxp autocorrelation matrix: eye(2)
nxn cost matrix: eye(2)
mxm cost matrix: 1e-3*eye(2)
Start point: [-5; 5]
Radius of the target area: 1
Radius of the obstacle area: 1
Center of the target area: [0; 5]
Center of the obstacle area: [0; 2.5]
Domain of t(in terms of start:step:end): -5:(5/(1e4-1)):0
1th Parametric function(using t as the parameter): t
2th Parametric function(using t as the parameter): 5
Degree of the polyfit for the reference trajectory: 5

lqgRT_v2.m
Users can pass the arguments when invoking the lqgRT_v2 function in their own code. One example is the file lqgRT_v2_test.m. The order of the parameters of the lqgRT_v2 function is as follow:
T, num_steps, n, m, p, pMinusS, A, B, C, Sigma_w, Sigma_v, Q, R, start_point, rd_tar, rd_obs, target, obs, t, parametric_func, degree

lqgRT_v3.m
Users can set the inputs in a 21-line configuration file with the same order above. Each input appears on its own line. An empty input should still be on a single line. lqgRT_v3.m file can read the configuration file and substitute the missing values with defaults automatically. One example of the configuration file is configuration.txt. 

More examples can be found in examples.txt.