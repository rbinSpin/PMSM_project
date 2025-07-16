function output = A_auxi(x_tilde_auxi,x_hat,param)

x = x_tilde_auxi(1:3) + x_hat;
z = x_tilde_auxi(4);

output = [         -k2,   k1,  k11*x(2),     (1/z)*k11*x_tilde(2)*x_hat(3);
          -k5-k10*x(3),  -k4,       0,    -(1/z)*k10*x_tilde(3)*x_hat(1);
               k9*x(2),    0,     -k7,      (1/z)*k9*x_tilde(2)*x_hat(1);
                     0,    0,       0,                              -eta];

end

