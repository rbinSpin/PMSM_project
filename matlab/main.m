% 建立參數 struct
param = struct();
param.J = 0.00782;
param.B = 0.02277;
param.R = 2.5;
param.Ld = 18e-3;
param.Lq = 32e-3;
param.lambda_m = 0.30175;
param.p = 6;
param.omega_b = 628;
param.Ib = 6.8;
param.Vb = 89;

% 預先計算 PU 常數，加入 param 結構
param.k1  = (3/2)*(1/param.J)*(param.p^2/4)*param.lambda_m*(1/param.omega_b);
param.k2  = param.B / param.J;
param.k3  = (param.p/2)*(1/param.J)*(1/param.omega_b);
param.k4  = param.R / param.Lq;
param.k5  = param.lambda_m / param.Lq * param.omega_b / param.Ib;
param.k6  = 1 / param.Lq * param.Vb / param.Ib;
param.k7  = param.R / param.Ld;
param.k8  = 1 / param.Ld * param.Vb / param.Ib;
param.k9  = param.Lq / param.Ld * param.omega_b;
param.k10 = param.Ld / param.Lq * param.omega_b;
param.k11 = (3/2)*(1/param.J)*(param.p^2/4)*(param.Ld - param.Lq)*param.Ib^2 / param.omega_b;

% 初始條件：tilde_x = x - x_hat
x0 = [0.0; 0.0; 0.0]; % 初始誤差

% 模擬時間
T = 0.001;
tspan = [0, 7*T];

% 給予動態的 desired state
% 設計行程
omega_e = @(t) ...
    (t < T) .* 0 + ...
    (t >= T & t < 2*T) .* (0.5 * (t - T) / T) + ...
    (t >= 2*T & t < 3*T) .* 0.5 + ...
    (t >= 3*T & t < 4*T) .* (0.5 - (t - 3*T)/T) + ...
    (t >= 4*T & t < 5*T) .* (-0.5) + ...
    (t >= 5*T & t < 6*T) .* (-0.5 + 0.5 * (t - 5*T)/T) + ...
    (t >= 6*T) .* 0;
% 呼叫 mtpa 函式
x_hat_func = @(t) x_hat_mtpa(omega_e(t), param);

% 模擬：使用 struct 傳遞
options = odeset('MaxStep', 1e-3);  % 限制最大步長為 0.001 秒
[t, x] = ode45(@(t, x) error_dynamics(t, x, x_hat_func(t), param), tspan, x0, options);

% 1 畫誤差狀態圖（保留原本的）
figure;
plot(t, x, 'LineWidth', 1.5)
legend('$\tilde{\omega}_e$', '$\tilde{i}_q$', '$\tilde{i}_d$', ...
       'Interpreter', 'latex')
xlabel('Time (s)')
ylabel('Error states')
title('SDRE 控制下誤差狀態演化')
grid on

% 2 計算實際狀態與目標狀態
x_real = zeros(size(x));
x_target = zeros(size(x));
w = zeros(size(x));

for i = 1:length(t)
    w(i) = omega_e(t(i));
    xh = x_hat_func(t(i));
    x_real(i,:) = x(i,:) + xh.';
    x_target(i,:) = xh.';
end

% 3 畫出轉速追蹤比較圖
figure;
plot(t, x_real(:,1), 'b', t, x_target(:,1), 'r--', 'LineWidth', 2)
legend('$\omega_e$ actual', '$\omega_e$ reference', 'Interpreter', 'latex')
xlabel('Time (s)')
ylabel('$\omega_e$ (PU)', 'Interpreter', 'latex')
title('實際轉速與目標轉速比較')
grid on

% 4 畫出電流追蹤圖 iq 與 id
figure;
subplot(2,1,1)
plot(t, x_real(:,2), 'b', t, x_target(:,2), 'r--', 'LineWidth', 2)
legend('$i_q$ actual', '$i_q$ reference', 'Interpreter', 'latex')
ylabel('$i_q$ (PU)', 'Interpreter', 'latex')
title('電流 i_q 追蹤比較')

subplot(2,1,2)
plot(t, x_real(:,3), 'b', t, x_target(:,3), 'r--', 'LineWidth', 2)
legend('$i_d$ actual', '$i_d$ reference', 'Interpreter', 'latex')
xlabel('Time (s)')
ylabel('$i_d$ (PU)', 'Interpreter', 'latex')
title('電流 i_d 追蹤比較')
grid on