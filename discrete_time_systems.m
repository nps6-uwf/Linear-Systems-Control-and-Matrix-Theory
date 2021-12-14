% This code shows how to simulate output feedback controllers in discrete-time

clear; clc;

% Input: Continuous time system matrices.
Ac = [0 1; 0 0];
Bc = [0;1];
C = [1 0];
D = 0;
n = size(Ac,2);
p = size(Bc,2); % number of inputs
m = size(C,1);


T = 0.1; % sampling time

% discretize the matrices. If they are already for discrete-time, DO NOT
% discretize again!
A = expm(Ac*T);
B = [T^2/2; T];

% Design the matrices F and K using the desired criteria (deadbeat response) as shown in
% lecture slides
F = 1/T^2*[-1 -3*T/2];
K = [2; 1/T];

% Closed-loop matrices using the states as x and e (= x - \hat{x}) -- see
% slide 8 in lecture 22
Acl = [A+B*F -B*F; zeros(n) A-K*C];
Bcl = [B; zeros(n,p)];
Ccl = [C+D*F zeros(size(C))];
Dcl = D;

% Simulate the closed loop system from any initial condition simply using a
% for loop:
nIter = 10; % The settling time is 2T = 4 iterations.
X = zeros(2*n,nIter); % Initialize the closed loop state
Y = zeros(m,nIter); % observations vector

x0 = [1;1]; % initial state
e0 = [5;3]; % initial error = x - xhat

X(:,1) = [x0;e0];

for i = 1:nIter-1
    X(:,i+1) = Acl*X(:,i);
    Y(:,i) = Ccl*X(:,i);
end

% Plot
plot(1:nIter, X(1,1:nIter), 'xr', 1:nIter, X(2,1:nIter), 'ob')
ylabel('States')
xlabel('Iteration')
legend('x_1','x_2')


