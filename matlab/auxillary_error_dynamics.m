function dx = auxillary_error_dynamics(~, x_tilde_auxi, u_tilde, A_auxi, B_auxi)
 % 
 % 1. functins depend on t: A_auxi(t), x_tilde_auxi                         
 % 2. Note that in ode45 u_tilde is constant because we don't change the    
 %    input voltage command until next time we command the driver.(10kHz)   
 % 3. Also, notice that B_auxi is always constant.                          
 % 
    
    % 誤差動態方程
    dx = A_auxi * x_tilde_auxi + B_auxi * u_tilde;
end
