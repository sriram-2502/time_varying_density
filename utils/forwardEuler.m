function [y_forward] = forwardEuler(y, deltaT)
    persistent y_next_initialized y_next
    
    % Initialize y_prev if it hasn't been initialized yet
    if isempty(y_next_initialized)
        y_next_initialized = true;
        y_next = 0; % Use y_prev=0 at t = 0
    end
    
    % Calculate the backward Euler approximation
    y_forward = (y_next - y) / deltaT;
    
    % Update y_prev for the next call
    y_next = y;
end
end