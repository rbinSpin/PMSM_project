pm_params_pu;


% 模擬時間
period = 1e-05;
T = 1;
ticks = 0:period:T;

% 給予動態的 desired state
% 設計行程
omega_e = zeros(size(ticks));
idx_06 = find(ticks <= 0.6);
omega_e(idx_06) = linspace(0, 0.16, length(idx_06));
omega_e(length(idx_06)+1:end) = 0.16;

% 呼叫 mtpa 函式
x_hat = zeros(3, length(ticks));

for i = 1:length(ticks)
    x_hat(:, i) = x_hat_mtpa(omega_e(i), param);
end

system_state = [0; 0; 0];
z = 0;

for i = 1:length(ticks)
    X_tilde = [system_state(1) - x_hat(1,i);
             system_state(2) - x_hat(2,i);
             system_state(3) - x_hat(3,i);
             z];
end