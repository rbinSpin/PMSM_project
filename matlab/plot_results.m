% plot_results.m

% 讀取數據
load('simulation_result.mat');  % 載入後會有 ticks, x_hat, u_tilde_log, x_tilde_auxi_log, x_log


%% 作圖
% 1. 需求速度與需求 id, iq
figure;
plot(ticks, x_hat(1, :), 'r-', ...
     ticks, x_hat(2, :), 'b--', ...
     ticks, x_hat(3, :), 'g-.');
xlabel('Time (s)');
ylabel('$\hat{x}$', 'Interpreter', 'latex', 'Rotation', 0);
legend('$\hat{\omega}$', '$\hat{i_q}$', '$\hat{i_d}$', 'Interpreter', 'latex');
title('reference');
grid on;



% 2. u_tilde to times
figure;
plot(ticks, u_tilde_log(1, :), 'r-', ticks, u_tilde_log(2, :), 'b--');
xlabel('Time (s)');
ylabel('$\tilde{u}$', 'Interpreter', 'latex', 'Rotation', 0);
legend('$\tilde{u_q}$', '$\tilde{u_d}$', 'Interpreter', 'latex');
title('Control Input History');
grid on;


% 3. x_tilde_auxi_log(ode45 final each iter) to times 
figure;
plot(ticks, x_tilde_auxi_log(1, :), 'r-', ...
     ticks, x_tilde_auxi_log(2, :), 'b--', ...
     ticks, x_tilde_auxi_log(3, :), 'g-.', ...
     ticks, x_tilde_auxi_log(4, :), 'k:');
xlabel('Time (s)');
ylabel('$\tilde{x_{auxi}}$', 'Interpreter', 'latex', 'Rotation', 0);
legend('$\tilde{\omega}$', '$\tilde{i_q}$', '$\tilde{i_d}$', '$z$', 'Interpreter', 'latex');
title('State Error Trajectories');
grid on;


% 4. x (system state) to times 
figure;
plot(ticks, x_log(1, :), 'r-', ...
     ticks, x_log(2, :), 'b--', ...
     ticks, x_log(3, :), 'g-.');
xlabel('Time (s)');
ylabel('$x$', 'Interpreter', 'latex', 'Rotation', 0);
legend('$\omega$', '$i_q$', '$i_d$', 'Interpreter', 'latex');
title('System State Trajectories');
grid on;