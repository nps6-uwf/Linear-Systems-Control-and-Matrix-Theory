% Nick Sebasco
% Date: 10/8/2021
% -------------------------------------------------------------------------
% Consider the continuous-time system:
% xdot = [1 1; 0 1]x + [0; 1]u
% If x(0) = [1; 1], find the control u(t) that will steer the state to
% [0;0] in 1 sec.  Using MATLAB calculate the response of the system and
% plot the trajectories of x1, x2, and u for t in [0, 1].

clc; clear;

% 1) Linear System Matrices
A = [1 1; 0 1];
B = [0; 1];
C = [1 1];
D = 0;

% Time interval
t0 = 0; 
tf = 1;
T = t0:0.01:tf;
T = T.';
x0 = [1; 1]; % initial condition
xf = [0; 0]; % final state - find u(t), which drives system here.

syms s t TF;
eAt = ilaplace(inv(s*eye(2) - A)); % state transition matrix
ieAt = inv(eAt); % inverse state transition matrix
ieAtT = (ieAt).'; % inverse transpose state transition matrix

% 2) Constructing the controllability Gramian
% gram = int(ieAt * B * B.' * ieAtT,t, 0, TF); % gramian
gram = int((eAt\B) * B.' * ieAtT,t, 0, TF); % a more numerically stable computation of gramian.
gram01 = subs(gram, TF, 1);      % gramian evaluated at Tf = 1

% 3) finding u(t)
% from lecture 9:
% The pair(A;B)is controllable (reachable) on[0;tf]
% if and only if the controllability Gramian Wc(0;tf)
% is positive definite.  From the proof of necessity an expression is
% derived for u(t):
% u(t) = transpose(B) * transpose(inverse(e^(At))) * inverse(gramian[0, tf]) * (-x0)

% u = (B.' * eAtT) * inv(gram01) * (-1 * x0);
u = (B.' * ieAtT) * (gram01\(-1 * x0)); % swap inv with \, more numerically stable

U = double(subs(u, t, T)); % convert sym matrix to double data type
SYS = ss(A,B,C,D);
%step(SYS);

[Y,T,X] = lsim(SYS, U, T,x0);
% plot(T,Y);
plot(T,X);




