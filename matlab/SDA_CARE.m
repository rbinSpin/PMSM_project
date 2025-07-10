%%
function [Q_hat_new,G]=SDA_CARE(A,B,Q,R)
%Solves the continuous Algebraic Riccati Equation by SDA
%Input A,B,Q,R:matices in the continuous-time algebraic Riccati equation
%A'*X+X*A-X*B*inv(R)*B'*X+Q=0
%
%Outputs:
%X:solution to the Riccati equation
%G=B*inv(R)*B'

Q_res=1000;

iteration_times = 0;

%max_iter=100;
max_iter=500;
res_tol=1e-16;
r=2.4;
%r=9.248791359444335;


%I = eye(size(A,1));
I = eye(size(A));
G = B*inv(R)*B';

A_r = A - (r*I);
IA_r=I/(A_r);

IA_r_multipy_G=IA_r*G;
IW_r=I/(A_r' + Q*IA_r_multipy_G);
%solve CARE with SDA
A_hat_last = I + 2*r*IW_r';
G_hat_last = 2*r*IA_r_multipy_G*IW_r;
Q_hat_last = 2*r*IW_r*Q*IA_r;
Q_hat_new = zeros(length(Q),length(Q));
while Q_res>res_tol && iteration_times<max_iter

    iteration_times = iteration_times + 1;

    %reduce redundent calculation by pre-calculating repeated terms
    inv_I_plus_G_H = I/(I + G_hat_last * Q_hat_last);
    transpose_A_hat_last = transpose(A_hat_last);

    %update
    A_hat_new = A_hat_last * inv_I_plus_G_H  * A_hat_last;
    G_hat_new = G_hat_last + (A_hat_last  * inv_I_plus_G_H* G_hat_last * transpose_A_hat_last);
    Q_hat_new = Q_hat_last + (transpose_A_hat_last * Q_hat_last* inv_I_plus_G_H  * A_hat_last);

    %delta_H matrix norms
    delta_Q=Q_hat_last-Q_hat_new;
    Q_res=norm(delta_Q)/norm(Q_hat_last);

    %prepare next iteration

    A_hat_last = A_hat_new;
    G_hat_last = G_hat_new;
    Q_hat_last = Q_hat_new;

end

end