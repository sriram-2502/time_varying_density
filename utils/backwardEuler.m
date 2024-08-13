function [y_backward] = backwardEuler(y, deltaT)
    persistent y_prev_initialized y_prev
    
    % Initialize y_prev if it hasn't been initialized yet
    if isempty(y_prev_initialized)
        disp('setting zero for first iteration in backward euler')
        y_prev_initialized = true;
        y_prev = 0; % Use y_prev=0 at t = 0
    end
    
    % Calculate the backward Euler approximation
    y_backward = (y - y_prev) / deltaT;
    
    % Update y_prev for the next call
    y_prev = y;
end