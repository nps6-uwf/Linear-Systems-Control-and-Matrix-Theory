% This code shows how to simulate output feedback controllers 
clear;
close all;
clc;

% Original system from the example
A = [0 1;1 -1];
B = [0; 1];
C = [1 0];
D = 0; % in this example

n = size(A,2);
m = size(C,1);

%% #1: Output-feedback Feedforward Controller
% u = F\hat{x} + Nr, \hat{x} = A\hat{x} + Bu + K(y - C\hat{x})

%Step 1: Design F, K and N as shown in the lecture.

F = [-2 -1];
K = [9; 17];
N = -C/(A+B*F)*B;

% Check if the closed loop eigenvalues are acceptable
eig(A+B*F)
eig(A-K*C)

% Step 2: Simulate this output feedback controller
% Need closed loop A, B, C, D matrices

Acl = [A B*F; K*C A-K*C+B*F]; % when we use the states as x and xhat.
Bcl = [B*N; B*N];
Ccl = [C zeros(size(C))];
Dcl = 0;

% Step 3: define the continuous time sys object and use lsim
sys = ss(Acl, Bcl, Ccl, Dcl);

t = 0:0.01:10;

t = t(:); % ensures that t is column vector.

r = ones(length(t),1);

[Y, T, X] = lsim(sys, r, t);

% plot
figure(1)
plot(T,Y)
hold on
plot(t,r,'--r')
hold on


%% #2: Output-feedback Integral Controller
% u = F1\hat{x} + F2*z, \dot{z} = r - y, \hat{x} = A\hat{x} + Bu + K(y - C\hat{x})

%Step 1: Design F1, F2 and K as shown in the lecture.

F1 = [-6 -3];
F2 = 2;
K = [9; 17];

% Check if the closed loop eigenvalues are acceptable
eig(A+B*F1)
eig(A-K*C)

% Step 2: Simulate this output feedback controller

% Need closed loop A, B, C, D matrices. In this example, we can use the
% states as x, z and the error e = x - \hat{x}. This allows us to use the
% closed loop equations from slide 22 of Lecture 21.

Acl = [A+B*F1 B*F2 -B*F1;
       -C zeros(m) zeros(m,n);
       zeros(n) zeros(n,m) A-K*C]; % when we use the states as x, z, and e.
Bcl = [zeros(n,m); eye(m); zeros(n,m)];
Ccl = [C zeros(m) zeros(m,n)];
Dcl = 0;

% Step 3: define the continuous time sys object and use lsim
sys = ss(Acl, Bcl, Ccl, Dcl);

t = 0:0.01:10;

t = t(:); % ensures that t is column vector.

r = ones(length(t),1);

[Y2, T2, X2] = lsim(sys, r, t);

% plot
figure(1)
plot(T2,Y2)
ylim([0,max(r)+0.1])
xlabel('Time')
ylabel('Output y(t)')
legend('Feedforward','Integral control')


