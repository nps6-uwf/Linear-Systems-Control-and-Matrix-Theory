%Author: Nick Sebasco

%Determine F so that the eigenvalues of A+BF are at -1,-2, and -2+-2j.
%Use the following methods to choose F:
%(a) transform (A,B) into controller form
%(b) take u2 to be 0, transform (A,B) to controller form
%(c) take u1 to be 0, transform (A,B) to controller form
%(d) Assign closed loop eigenvectors such that the first component of the
%eigenvector associated with -1 is 0.
%(e) Assign closed loop eigenvectors such that the first component of the
%eigenvector associated with -2+-2j is 0.
%(f) Use place command

clc; clear;

A = [0 1 0 0;0 0 1 0;0 0 0 1;1 -1 1 1], B = [1 0;0 1;0 0;0 1]

% (a)
CM = ctrb(A,B)

% The first four columns are linearly independent. mu_1 = mu_2 = 2.
CM_bar = [CM(:,1) CM(:,3) CM(:,2) CM(:,4)]

CM_bar_inv = inv(CM_bar)

Q = [CM_bar_inv(2,:); CM_bar_inv(2,:)*A; CM_bar_inv(4,:); CM_bar_inv(4,:)*A]

P = inv(Q); Ac = Q*A*P, Bc = Q*B

Am = [Ac(2,:);Ac(4,:)], Bm = [Bc(2,:);Bc(4,:)]

% Find the desired characteristic equation
d1 = [1 1]; d2 = [1 2]; d3 = [1 4 8]; d4 = conv(d1,d2); d = conv(d3,d4)
roots(d)

Amd = [0 0 1 0;-16 -32 -22 -7]

Fc = inv(Bm)*(Amd - Am), FA = Fc*Q

% Check
eig(A+B*FA)

% (b)
CM1 = ctrb(A,B(:,1)), rank(CM1)

CM1_inv = inv(CM1)

Q1 = [CM1_inv(4,:);CM1_inv(4,:)*A;CM1_inv(4,:)*A*A;CM1_inv(4,:)*A*A*A]

P1 = inv(Q1); Ac1 = Q1*A*P1, Bc1 = Q1*B(:,1)

Am1 = Ac1(4,:), Amd1 = [-16 -32 -22 -7], Fc1 = Amd1-Am1

F1 = Fc1*Q1

FB = [F1;0 0 0 0]

% Check
eig(A+B*FB)

% (c)
CM2 = ctrb(A,B(:,2)), rank(CM2)

CM2_inv = inv(CM2)

Q2 = [CM2_inv(4,:);CM2_inv(4,:)*A;CM2_inv(4,:)*A*A;CM2_inv(4,:)*A*A*A]

P2 = inv(Q2); Ac2 = Q2*A*P2, Bc2 = Q2*B(:,2)

Am2 = Ac2(4,:), Amd2 = [-16 -32 -22 -7], Fc2 = Amd2-Am2

F2 = Fc2*Q2

FC = [0 0 0 0; F2]

% Check
eig(A+B*FC)

% (d)
S1 = [-1*eye(4)-A, B], S2 = [-2*eye(4)-A, B], S3 = [(-2+i*2)*eye(4)-A, B]

Y1 = null(S1), Y2 = null(S2), Y3 = null(S3)

M1 = Y1(1:4,:); D1 = -Y1(5:6,:);
M2 = Y2(1:4,:); D2 = -Y2(5:6,:);
M3 = Y3(1:4,:); D3 = -Y3(5:6,:);

v1 = M1(:,1)-(M1(1,1)/M1(1,2))*M1(:,2)

v2 = M2(:,1); v3 = M3(:,1); v4 = conj(v3);
V = [v1 v2 v3 v4], rank(V)

xi1 = [1;-M1(1,1)/M1(1,2)], xi2= [1;0], xi3 = [1;0]

q1 = D1*xi1; q2 = D2*xi2; q3 = D3*xi3; q4 = conj(q3); Q = [q1 q2 q3 q4]
 
FD = Q*inv(V)

FD = real(FD)
% Check
[VV,EE] = eig(A+B*FD)

% The fourth column of VV is a multiple of v1
% (e)
v1 = M1(:,1); v2 = M2(:,1); v3 = M3(:,1)-(M3(1,1)/M3(1,2))*M3(:,2), v4 = conj(v3)

V = [v1 v2 v3 v4], rank(V)

xi3 = [1;-M3(1,1)/M3(1,2)], xi1= [1;0], xi2 = [1;0]

q1 = D1*xi1; q2 = D2*xi2; q3 = D3*xi3; q4 = conj(q3); Q = [q1 q2 q3 q4]
FE = Q*inv(V)
FE = real(FE)

% Check
[VV,EE] = eig(A+B*FE)

% The first column of VV is a multiple of v3, as shown below.
(v3(2,1)/VV(2,1))*VV(:,1)
% (f)
FF = - place(A,B,[-1;-2;-2+2*i;-2-2*i])
% Check
eig(A+B*FF)
