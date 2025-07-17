function output = A_auxi_fun(x_tilde_auxi,x_hat,param)

k1 = param.k1;
k2 = param.k2;
k3 = param.k3;
k4 = param.k4;
k5 = param.k5;
k6 = param.k6;
k7 = param.k7;
k8 = param.k8;
k9 = param.k9;
k10 = param.k10;
k11 = param.k11;

eta = param.eta;

x = x_tilde_auxi(1:3) + x_hat;
z = x_tilde_auxi(4);

output = [         -k2,   k1,  k11*x(2),     (1/z)*k11*x_tilde_auxi(2)*x_hat(3);
          -k5-k10*x(3),  -k4,       0,      -(1/z)*k10*x_tilde_auxi(3)*x_hat(1);
               k9*x(2),    0,     -k7,        (1/z)*k9*x_tilde_auxi(2)*x_hat(1);
                     0,    0,       0,                                     -eta];

end

