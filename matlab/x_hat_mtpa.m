function x_hat = x_hat_mtpa(omega_e, param)
    % param 應該是一個 struct，內含 B, J, Ld, Lq, lambda_m, p

    B = param.B;
    J = param.J;
    Ld = param.Ld;
    Lq = param.Lq;
    lambda_m = param.lambda_m;
    p = param.p;

    % Step 1: 給定 omega_e（PU），計算 Te（PU）
    Te = B * omega_e;  % 忽略負載力矩 TL

    % Step 2: MTPA cubic: a * iq^3 + b * iq - Te = 0
    a = (3/2)*(p/2)*(Ld - Lq)^2 / lambda_m;
    b = (3/2)*(p/2)*lambda_m;

    iq_roots = roots([a, 0, b, -Te]);
    iq_real = iq_roots(imag(iq_roots) == 0);  % 實根
    iq = iq_real(iq_real >= 0);              % 正實根

    if isempty(iq)
        iq = 0;
    else
        iq = iq(1);  % 選擇最小正實根（MTPA 一般只有一個）
    end

    % Step 3: 對應 MTPA id
    id = (Ld - Lq)/lambda_m * iq^2;

    % Step 4: 輸出目標狀態
    x_hat = [omega_e; iq; id];
end