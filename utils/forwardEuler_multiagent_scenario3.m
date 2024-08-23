function [x_euler, u_euler, x_dot] = forwardEuler_multiagent_vs_SFM(p, deltaT, ctrl_multiplier, dynamics, gradDensityHandles, c1, c2, c3, dyn_p, x_temp, agent_number, dens_bool)
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
        dens_bool = true;    end

    % Compute state derivatives and control inputs
    [x_dot, u_euler, isgoal, unicycle] = dynamics(deltaT, x_temp, ctrl_multiplier, gradDensityHandles, c1, c2, c3, p, dyn_p, agent_number, dens_bool);
    % Forward Euler integration
    x_euler = x_temp + deltaT * x_dot;


    % % compute states for RK4
    % step_size = 0.01;
    % % Step 1: Compute k1
    % [x_dot1, ~, ~, ~] = dynamics(deltaT, x_temp, ctrl_multiplier, gradDensityHandles, c1, c2, c3, c4, p, dyn_p, agent_number, dens_bool);
    % k1 = deltaT * x_dot1;
    % 
    % % Step 2: Compute k2
    % [x_dot2, ~, ~, ~] = dynamics(deltaT+(step_size/2), x_temp+(step_size/2)*k1, ctrl_multiplier, gradDensityHandles, c1, c2, c3, c4, p, dyn_p, agent_number, dens_bool);
    % k2 = deltaT * x_dot2;
    % 
    % % Step 3: Compute k3
    % [x_dot3, ~, ~, ~] = dynamics(deltaT+(step_size/2), x_temp+(step_size/2)*k2, ctrl_multiplier, gradDensityHandles, c1, c2, c3, c4, p, dyn_p, agent_number, dens_bool);
    % k3 = deltaT * x_dot3;
    % 
    % % Step 4: Compute k4
    % [x_dot4, ~, ~, ~] = dynamics(deltaT+step_size, x_temp+step_size*k3, ctrl_multiplier, gradDensityHandles, c1, c2, c3, c4, p, dyn_p, agent_number, dens_bool);
    % k4 = deltaT * x_dot4;
    % % RK4
    % x_euler = x_temp + (1/6) * (k1 + 2*k2 + 2*k3 + k4);

    if(unicycle)
        % Wrap theta to the range [-pi, pi]
        theta = x_euler(3);
        theta = atan2(sin(theta), cos(theta));
        x_euler(3) = theta;
    end         

    if(isgoal)
        disp(['----- Agent ', num2str(agent_number), ' reached goal -----'])
    end

    % Transpose and return variables
    x_euler = x_euler';
    x_dot = x_dot';
    u_euler = u_euler';
end
