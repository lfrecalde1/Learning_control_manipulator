% Learning dynamics

% Clean variables
clc, clear all, close all;

% Time definition
t_s = 0.01;
t_final = 60;
t = (0:t_s:t_final);

% Control gains scalar value
kp = 10;
wn = sqrt(kp);
kv = 6*1*wn;

% Value of P
max_value = (((kv^2)/2)-2)/(sqrt((kv^2)/(4)-1))

u = 10;


% Transfer funtion definitions
P = tf([1 (kv-u) kp-u], [0 0 1])
A = tf([1], [1 kv kp])
S = tf([0 0 1], [0 1 0])

% Operator contraction
aux = P*A

% PD behavior
A_d = c2d(A, t_s)

% Operator Function
aux_d = c2d(aux, t_s)
[num1d,den1d]=tfdata(aux_d,'v')

