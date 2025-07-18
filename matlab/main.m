clc; clear;

%% Get the param
pm_params_pu;

HUNDRED_kHz = true; TEN_kHz = false;
Q_ORIGIN = true; QZ0 = false;
SDA = true; ICARE = false;

SEL_PERIOD = HUNDRED_kHz;
SEL_Q      = Q_ORIGIN;
SEL_SOLVER = SDA;

%% Offline MTPA  
% 模擬時間
if SEL_PERIOD
    period = 1e-5;
else
    period = 1e-4;
end

tspan = [0 period];
T = 0.5;  
ticks = 0:period:T;  

% 初始化 omega_e  
omega_e = zeros(size(ticks));  

% 0 ~ 0.1 秒：0 ~ 0.16 線性增加  
idx1 = ticks <= 0.1;  
omega_e(idx1) = linspace(0, 0.16, sum(idx1));  

% 0.1 ~ 0.2 秒：保持 0.16  
idx2 = (ticks > 0.1) & (ticks <= 0.2);  
omega_e(idx2) = 0.16;  

% 0.2 ~ 0.32 秒：0.16 ~ -0.16 線性減少  
idx3 = (ticks > 0.2) & (ticks <= 0.32);  
omega_e(idx3) = linspace(0.16, -0.16, sum(idx3));  

% 0.32 ~ 0.4 秒：保持 -0.16  
idx4 = (ticks > 0.32) & (ticks <= 0.4);  
omega_e(idx4) = -0.16;  

% 0.4 ~ 0.5 秒：-0.16 ~ 0 線性增加  
idx5 = (ticks > 0.4) & (ticks <= 0.5);  
omega_e(idx5) = linspace(-0.16, 0, sum(idx5));  

% 呼叫 mtpa 函式
x_hat = zeros(3, length(ticks));

for i = 1:length(ticks)
    x_hat(:, i) = x_hat_mtpa(omega_e(i), param);
end

%% Simulation
% initialization
x = [0; 0; 0]; % system state [omega; iq; id]
z = 1;
ic = [x - x_hat(:, 1); z];  % 初始誤差

% for loop
h = waitbar(0, 'Please wait...');

% data buffer
n_loop = length(ticks);
u_tilde_log = zeros(2, n_loop);        % 2 是控制量維度
x_tilde_auxi_log = zeros(4, n_loop);   % 4 是狀態變數維度
x_log = zeros(3, n_loop);              % 3 是系統狀態維度

for i=1:length(ticks)
    % 計算誤差
    x_tilde = x - x_hat(:, i);
    x_tilde_auxi = [x_tilde; z];
    x_tilde_auxi_log(:, i) = x_tilde_auxi;

    % A_auxi(x_tilde_auxi,x_hat,param)
    A_auxi = A_auxi_fun(x_tilde_auxi,x_hat(:, i),param);
    
    % B_auxi: constant
    B_auxi = [        0,         0;
               param.k6,         0;
                      0,  param.k8;
                      0,         0];
    
    % Q, R 矩陣
    rate = sqrt(x_tilde(2)^2 + x_tilde(3)^2);
    if SEL_SOLVER
        Q = diag([5000, 1 + exp(-rate), 10000 + exp(-rate), 100]);  % z 的權重 100，可調
    else
        Q = diag([5000, 10000 + exp(-rate), 10000 + exp(-rate), 0]);
    end
    
    R = diag([1 1]);
    
    % 解 SDRE 控制律 (K)
    if SEL_Q
        [P_ss_care, ~] = SDA_CARE(A_auxi, B_auxi, Q, R);
    else
        [P_ss_care, ~] = icare(A_auxi, B_auxi, Q, R, zeros(size(B_auxi)), eye(size(A_auxi)));
    end

    u_tilde = -inv(R) * B_auxi' * P_ss_care * x_tilde_auxi;
    u_tilde_log(:, i) = u_tilde;

    
    % 輸入電壓指令並模擬 period
    opts = odeset('MaxStep', period, 'InitialStep', period, 'RelTol', 1e-3, 'AbsTol', 1e-6);
    [t,x_tilde_auxi] = ode45( ...
        @(t,x_tilde_auxi) auxillary_error_dynamics(t, x_tilde_auxi, u_tilde, x_hat(:, i), param, B_auxi), ...
        tspan, ...
        ic,...
        opts);
    
    % 更新 system state
    x_tilde_auxi = x_tilde_auxi(end, :).';  % 取最後一列，轉成 column vector
    x = x_tilde_auxi(1:3) + x_hat(:,i);
    x_log(:, i) = x;
    z = x_tilde_auxi(4);
    ic = x_tilde_auxi;

    waitbar(i/length(ticks), h, sprintf('Progress: %d%%', round(i/length(ticks)*100)));
end

close(h);


%% save the simulation data
save('simulation_result.mat', 'ticks','x_hat', 'u_tilde_log', 'x_tilde_auxi_log', 'x_log');
