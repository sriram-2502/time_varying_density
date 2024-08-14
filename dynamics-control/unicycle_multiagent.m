function [x_dot, u, isgoal] = unicycle_multiagent(deltaT, x, ctrl_multiplier, gradDensityHandles, c1, c2, c3, c4, p, single_int_p, agent_number, dens_bool)
    % Propagates the kinematic model with a control and updates with LQR control
    % Inputs:
    %   deltaT              : Time step
    %   x                   : Current state of the system (R^n)
    %   ctrl_multiplier     : Scaling value to increase control value input
    %   gradDensityHandles  : Handles to gradient density functions
    %   c1, c2, c3, c4      : Parameters for the gradient functions
    %   p                   : Navigation parameters (xd, rad_from_goal)
    %   single_int_p        : Single Integrator parameters
    %   agent_number        : Index of the agent
    %   dens_bool           : Boolean to enable density functions vs navigation functions
    % Outputs:
    %   x_dot               : The velocity state or control input (x_dot = u)
    %   u                   : Control input
    %   isgoal              : Boolean indicating if the goal is reached

    % Default arguments
    if nargin < 12
        dens_bool = true;
    end

    % Parameters
    rad_from_goal = p.rad_from_goal;
    saturation_vel = 2;
    saturation_omega = 1;
    Kp = 1;
    theta = x(3);
    isgoal = false;

    % Determine the goal position and control based on the agent number
    switch agent_number
        case 1
            xd = p.xd1;
            grad_density = gradDensityHandles.grad_density1;
        case 2
            xd = p.xd2;
            grad_density = gradDensityHandles.grad_density2;
        case 3
            xd = p.xd3;
            grad_density = gradDensityHandles.grad_density3;
        case 4
            xd = p.xd4;
            grad_density = gradDensityHandles.grad_density4;
        otherwise
            error('Invalid agent_number');
    end

    % Control logic based on the density function boolean
    if dens_bool
        % Switch control if near the goal
        if norm(x - xd) < rad_from_goal
            vel = 0;
            theta_tilda = 0;
            isgoal = true;
        else
            u_hat = ctrl_multiplier * grad_density(x, c1, c2, c3, c4);
            vel = norm(u_hat);
            theta_tilda = atan2(u_hat(2), u_hat(1));
        end
    else
        u_hat = -ctrl_multiplier * grad_phi_f(x);
        vel = norm(u_hat);
        theta_tilda = atan2(u_hat(2), u_hat(1));
    end

    % Compute state derivatives
    x1_dot = vel * cos(theta_tilda);
    x2_dot = vel * sin(theta_tilda);
    
    % Use backstepping to map single integrator control to unicycle model
    theta_tilda_dot = backwardEuler(agent_number, theta_tilda, deltaT);
    w = theta_tilda_dot - Kp * (theta - theta_tilda);
    theta_dot = w;

    % Control input
    u = [vel; w];
    x_dot = [x1_dot; x2_dot; theta_dot];

    % Saturation
    if abs(u(1)) > saturation_vel
        u(1) = saturation_vel * sign(u(1));
    end
    if abs(u(2)) > saturation_omega
        u(2) = saturation_omega * sign(u(2));
    end
end
