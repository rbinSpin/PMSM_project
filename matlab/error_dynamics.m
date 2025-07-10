function dx = error_dynamics(~, x_tilde, x_hat, param)

    % 還原實際狀態
    x = x_tilde + x_hat;
    omega_e = x(1);
    iq = x(2);
    id = x(3);

    % 提取參數（簡化符號）
    k1  = param.k1;
    k2  = param.k2;
    k3  = param.k3;
    k4  = param.k4;
    k5  = param.k5;
    k6  = param.k6;
    k7  = param.k7;
    k8  = param.k8;
    k9  = param.k9;
    k10 = param.k10;
    k11 = param.k11;

    % A(x)
    A = [ -k2,    k1,      k11 * iq;
          -k5,  -k4,    -k10 * id;
           k9 * iq, 0,     -k7 ];

    % B
    B = [ 0,   0;
          k6, 0;
          0,  k8 ];

    % SDRE 權重
    Q = diag([100, 1, 100]);
    R = eye(2);

    % 使用 SDA_CARE 求 Riccati 解
    [P, ~] = SDA_CARE(A, B, Q, R);  % 你自定義的 Riccati 求解器
    K = R \ (B' * P);               % 回授增益

    % 控制律
    u_tilde = -K * x_tilde;

    % 非線性補償項 g = (A(x) - A(x_hat)) * x_hat
    iq_hat = x_hat(2);
    id_hat = x_hat(3);
    omega_hat = x_hat(1);

    g = [
        k11 * (iq - iq_hat) * id_hat;
       -k10 * (id - id_hat) * omega_e;
        k9  * (iq - iq_hat) * omega_e
    ];

    % 誤差動態方程
    dx = A * x_tilde + B * u_tilde + g;
end
