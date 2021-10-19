% Nick Sebasco
% 10/19/2021
% Use a similarity transformation so that the pair (A, B) becomes
% controllable.

clc;clear;

A = [0 0 1 0; 0 0 1 0; 0 0 0 0; 0 0 0 -1];
B = [0 1; 0 0; 1 0; 0 0];
[~,bn] = size(B);

% Controllability test:
% If q = rank(controllability matrix) < n -> uncontrollable
q = rank([B A*B A.^2*B]);

% Choose first 3 linear independent columns to construct P:
P = [0 1 1; 0 0 1; 1 0 0; 0 0 0];
% Now concatenate n - q: arbitrary columns such that rank(P) = 4:
cn = [0;0;0;1]; % an obvious choice.
P = [P cn];

% Construct the new controllable pair: (A11_hat, B1_hat)
Bhat = inv(P)*B;
Ahat = inv(P)*A*P;

A11_hat = Ahat(1:q, 1:q)
[nhat,~] = size(A11_hat);
B1_hat = Bhat(1:q, 1:bn)

% Verify (A11_hat, B1_hat) is controllable:
rank([B1_hat A11_hat*B1_hat A11_hat.^2*B1_hat]) == nhat



