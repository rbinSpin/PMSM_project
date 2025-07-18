% plot_results.m
clear; clc;
SAVE_PICTURES = true;

% 讀取數據
load('simulation_result.mat');  % 載入後會有 ticks, x_hat, u_tilde_log, x_tilde_auxi_log, x_log


%% 作圖
% 1. x_hat to ticks
figure;
plot(ticks, x_hat(1, :), 'r-', ...
     ticks, x_hat(2, :), 'b--', ...
     ticks, x_hat(3, :), 'g-.');
xlabel('Time (s)');
ylabel('$\hat{x}$', 'Interpreter', 'latex', 'Rotation', 0);
legend('$\hat{\omega} (pu)$', '$\hat{i_q} (pu)$', '$\hat{i_d} (pu)$', 'Interpreter', 'latex');
title('reference');
grid on;

if SAVE_PICTURES
    saveas(gcf, 'matlab_figures/fig1_x_hat.png');
end



% 2. u_tilde to ticks
figure;
plot(ticks, u_tilde_log(1, :), 'r-', ticks, u_tilde_log(2, :), 'b--');
xlabel('Time (s)');
ylabel('$\tilde{u}$', 'Interpreter', 'latex', 'Rotation', 0);
legend('$\tilde{u_q} (pu)$', '$\tilde{u_d} (pu)$', 'Interpreter', 'latex');
title('Control Input History');
grid on;

if SAVE_PICTURES
    saveas(gcf, 'matlab_figures/fig2_u_tilde.png');
end


% 3. x_tilde_auxi_log(ode45 final each iter) to ticks
figure;
plot(ticks, x_tilde_auxi_log(1, :), 'r-', ...
     ticks, x_tilde_auxi_log(2, :), 'b--', ...
     ticks, x_tilde_auxi_log(3, :), 'g-.', ...
     ticks, x_tilde_auxi_log(4, :), 'k:');
xlabel('Time (s)');
ylabel('$\tilde{x_{auxi}}$', 'Interpreter', 'latex', 'Rotation', 0);
legend('$\tilde{\omega} (pu)$', '$\tilde{i_q} (pu)$', '$\tilde{i_d} (pu)$', '$z$', 'Interpreter', 'latex');
title('State Error Trajectories');
grid on;

if SAVE_PICTURES
    saveas(gcf, 'matlab_figures/fig3_x_tilde_auxi_log.png');
end


% 4. x (system state) to ticks 
figure;
plot(ticks, x_log(1, :), 'r-', ...
     ticks, x_log(2, :), 'b--', ...
     ticks, x_log(3, :), 'g-.');
xlabel('Time (s)');
ylabel('$x$', 'Interpreter', 'latex', 'Rotation', 0);
legend('$\omega (pu)$', '$i_q (pu)$', '$i_d (pu)$', 'Interpreter', 'latex');
title('System State Trajectories');
grid on;

if SAVE_PICTURES
    saveas(gcf, 'matlab_figures/fig4_x_log.png');
end