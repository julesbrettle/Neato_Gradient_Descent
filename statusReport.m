function statusReport(s)
    global lambda stepNum grad r_current theta exp_current_r position
    
    fprintf('rostime:%7.0f    ', rostime('now').Sec)
    fprintf('stepNum:%3.0f    ', stepNum)
    fprintf('lambda: %4.5f     ', lambda)
    fprintf('grad: [%4.5f; %4.5f]     ', grad(1),grad(2))
    fprintf('\n            r_current: [%4.2f, %4.2f] theta: %2.5fpi',r_current(1),r_current(2),theta/pi())
    fprintf('\n             position: [%4.2f, %4.2f,      %2.5fpi]',position(1),position(2),position(3)/pi())
    fprintf('\n            -- %s  \n',s)
    
    
end