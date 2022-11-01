%% Code Manipulator 2DOF
% Clean variables
clc, clear all, close all;

% Time defintion variables
t_s = 0.01;
t_final = 60;
t = (0:t_s:t_final);

g = 9.8;
% System parameters L1
b1 = 1;
m1 = 0.8;
l1 = 1;
Iz1= 0.5;

% System parameters L2
b2 = 1;
m2 = 0.8;
l2 = 1;
Iz2= 0.5;


L1 = [b1 , m1, l1, Iz1];
L2 = [b2 , m2, l2, Iz2];

% Initial conditions system       
q = zeros(4, length(t)+1);
q(:, 1) = [0*pi/180;...
           0;...
           0;...
           0];

% Constant defintion
constans = [g, t_s];

% Robot definition
robot = manipulator_system(L1, L2, constans, q(:,1));


% Desired angles of the system
qd = [90*pi/180*ones(1, length(t));...
       0*pi/180*ones(1, length(t))];
   
qdp = [0*pi/180*ones(1, length(t));...
       0*pi/180*ones(1, length(t))];
   
qdpp = [0*pi/180*ones(1, length(t));...
        0*pi/180*ones(1, length(t))];

% Control gains
kp = 20;
wn = sqrt(kp);
kv = 4*1*wn;

% Learning variable
max_value = (((kv^2)/2)-2)/(sqrt((kv^2)/(4)-1));

gamma = 0.1;

% Learning system
% Transfer funtion definitions
P = tf([1 (kv-gamma) kp-gamma], [0 0 1]);
A = tf([1], [1 kv kp]);
S = tf([0 0 1], [0 1 0]);

% Operator contraction
aux = S*P*A;

% Operator Function
aux_d = c2d(aux, t_s);
[num1d, den1d] = tfdata(aux_d,'v');


% PD control Gains
K1 = kp*eye(2);
K2 = kv*eye(2);

% Controller definition
control = controller(K1, K2, num1d, den1d, robot);

% Control vector empty
u = zeros(2, length(t));

% Control vector error definition
qe = zeros(2, length(t));
qep = zeros(2, length(t));

% External torque of the system
T_extern = zeros(2, length(t));


for k = 1:length(t)
    % Control vector
    qe(:, k) = qd(:, k) - robot.get_positions();
    qep(:, k) = qdp(:, k) - robot.get_velocities();
    
    % Control it itslef depents of the variable robot
    u(:, k) = control.get_control_PD(qd(:, k), qdp(:, k));
    
    % System evolution
    q(:, k+1) = robot.system_f(u(:, k), T_extern(:, k));
    
    
end

for k = 1:10:length(t)
    drawpend2(q(:, k), m1, m2, 0.3, l1, l2);
end

save("Data_PD.mat", "t", "q", "qd", "qdp", "qe", "qep", "u", "T_extern","L1", "L2")