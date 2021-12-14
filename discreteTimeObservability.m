% Consider the discrete time system:
% x(k+1) = [1 2; 0 1]x(k) + [0;1]u(k)
% y(k) = [1 1]x(k)
% using only measurements of y, find u(k) for k=0,1,2,3,4,5 that steer the
% state of the system from any unknown initial state x(0) to x(6)=[0;0].

% [y(0);y(1)] = [1 1; 1 3]x(0)
% x(0) = 1/2[3 -1; -1 1][y(0);y(1)] = 1/2[3y(0)-y(1);-y(0)+y(1)]
% x(2) = A^2x(0) = 1/2[1 4; 0 1][3y(0)-y(1);-y(0)+y(1)] = 1/2[-y(0)+3y(1);-y(0)+y(1)]
%-A^2x(2) = C[u(3); u(2)];
% [u(3); u(2)] = -C^-1A^2x(2)

clc; clear; close;

A = [1 2; 0 1];
B = 0;1;
C = [1 1];
x(:,1) = [1;2];
y = zeros(6,1);
u = zeros(6,1);

% generate observations
for k = 1:2
    x(:,k+1) = A*x(:,k) + B*u(k)
    y(k) = C*x(:,k)
end

u(3) = 5/4*y(1) - 7/4*y(2);
u(4) = -3/4*y(1) + 5/4*y(2);
u(5) = 0;
u(6) = 0;

for k = 3:6
    x(:,k+1) = A*x(:,k) + B*u(k);
    y(k) = C*x(:,k);
end

x(:,k)