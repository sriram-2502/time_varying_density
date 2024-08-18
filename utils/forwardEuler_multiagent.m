function [x_euler, u_euler, x_dot] = forwardEuler_multiagent(p, deltaT, ctrl_multiplier, dynamics, gradDensityHandles, c1, c2, c3, c4, dyn_p, x_temp, agent_number, dens_bool)
    % Forward Euler integration to propagate dynamics and return states
    %
    % Inputs:
    %   p                   : Parameters (initial conditions x0, goal state xd, radius from goal)
    %   deltaT              : Discrete step size
    %   ctrl_multiplier     : Multiplier on control
    %   dynamics            : Function handle for dynamics
    %   gradDensityHandles  : Handles to gradient density functions
    %   c1, c2, c3, c4      : Parameters for gradient functions
    %   dyn_p               : Dynamic system parameters (A, B matrices, LQR Feedback Gain)
    %   x_temp              : Current state
    %   agent_number        : Index of the agent
    %   dens_bool           : Boolean to enable density functions vs navigation functions
    %
    % Outputs:
    %   x_euler             : Forward integrated states
    %   u_euler             : Control input trajectory
    %   x_dot               : State derivative trajectory

    % Default argument
    if nargin < 13
        dens_bool = true;
    end

    % Compute state derivatives and control inputs
    [x_dot, u_euler, isgoal, unicycle] = dynamics(deltaT, x_temp, ctrl_multiplier, gradDensityHandles, c1, c2, c3, c4, p, dyn_p, agent_number, dens_bool);
    
    
    if(~isgoal)
        % Forward Euler integration
        x_euler = x_temp + deltaT * x_dot;
        if(unicycle)
            % Wrap theta to the range [-pi, pi]
            theta = x_euler(3);
            theta = atan2(sin(theta), cos(theta));
            x_euler(3) = theta;
        end         
    else
        % Agent has reached the goal
        disp(['----- Agent ', num2str(agent_number), ' reached goal -----'])
        x_euler = x_temp;
        x_dot = zeros(size(x_temp));
        u_euler = zeros(2, 1);
    end

    % Transpose and return variables
    x_euler = x_euler';
    x_dot = x_dot';
    u_euler = u_euler';
end
