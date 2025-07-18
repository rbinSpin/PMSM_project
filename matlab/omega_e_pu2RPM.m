clc; clear;

%% Get the param
pm_params_pu;

% 設定基本參數
omega_base = param.wb;  % 例：基準電氣角速度 rad/s (2π × 50Hz)
pole_pairs = param.Polepairs;       % 極對數

fprintf('--- omega_pu / RPM 轉換工具 ---\n');
fprintf('基準 omega = %.2f rad/s, 極對數 = %d\n\n', omega_base, pole_pairs);
fprintf('請選擇轉換方向：\n');
fprintf('1. omega_pu ➔ RPM\n');
fprintf('2. RPM ➔ omega_pu\n');
fprintf('q. 離開\n');

while true
    
    choice = input('選擇 (1/2/q): ', 's');
    
    if strcmpi(choice, 'q')
        disp('程式結束。');
        break;
    end
    
    switch choice
        case '1'  % omega ➔ RPM
            user_input = input('請輸入 omega_pu (pu): ', 's');
            omega_pu = str2double(user_input);
            if isnan(omega_pu)
                disp('❌ 輸入無效，請重新輸入數字。');
                continue;
            end
            omega_rad = omega_pu * omega_base;
            rpm = omega_rad * 60 / (2 * pi) / pole_pairs;
            fprintf('✅ omega_pu = %.4f ➔ RPM = %.2f\n\n', omega_pu, rpm);
            
        case '2'  % RPM ➔ omega
            user_input = input('請輸入 RPM: ', 's');
            rpm = str2double(user_input);
            if isnan(rpm)
                disp('❌ 輸入無效，請重新輸入數字。');
                continue;
            end
            omega_rad = rpm * 2 * pi * pole_pairs / 60;
            omega_pu = omega_rad / omega_base;
            fprintf('✅ RPM = %.2f ➔ omega_pu = %.4f\n\n', rpm, omega_pu);
            
        otherwise
            disp('❌ 無效的選項，請選擇 1, 2 或 q。');
    end
end