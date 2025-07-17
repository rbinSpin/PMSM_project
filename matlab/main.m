clc; clear;
%% Get the param
pm_params_pu;

%% Offline MTPA
% 模擬時間
period = 1e-05;
T = 0.1;
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

%% Simulation initiation

x = [0; 0; 0]; % system state [omega; iq; id]
z = 1;
ic = [x - x_hat(:, 1); z];  % 初始誤差

%% for loop
h = waitbar(0, 'Please wait...');

for i=1:length(ticks)
    % 計算誤差
    x_tilde = x - x_hat(:, i);
    x_tilde_auxi = [x_tilde; z];

    % A_auxi(x_tilde_auxi,x_hat,param)
    A_auxi = A_auxi_fun(x_tilde_auxi,x_hat,param);
    
    % B_auxi: constant
    B_auxi = [        0,         0;
               param.k6,         0;
                      0,  param.k8;
                      0,         0];
    
    % Q, R 矩陣
    rate = sqrt(x_tilde(2)^2 + x_tilde(3)^2);
    Q = diag([5000, 1 + exp(-rate), 10000 + exp(-rate), 100]);  % z 的權重 100，可調
    R = diag([1 1]);
    
    % 解 SDRE 控制律 (K)
    [P_ss_care, ~] = SDA_CARE(A_auxi, B_auxi, Q, R);
    u_tilde = -inv(R) * B_auxi' * P_ss_care * x_tilde_auxi;
    
    % 輸入電壓指令並模擬 1e-5s 
    tspan = [0 1e-5];
    [t,x_tilde_auxi] = ode45( ...
        @(t,x_tilde_auxi) ...
        auxillary_error_dynamics(t, x_tilde_auxi, u_tilde, ...
        A_auxi, B_auxi), ...
        tspan, ...
        ic);
    
    % 更新 system state
    x_tilde_auxi = x_tilde_auxi(end, :).';  % 取最後一列，轉成 column vector
    x = x_tilde_auxi(1:3) + x_hat(:,i);
    z = x_tilde_auxi(4);
    ic = x_tilde_auxi;

    waitbar(i/length(ticks), h, sprintf('Progress: %d%%', round(i/length(ticks)*100)));
end

close(h);