function [x_dot, u, isgoal, unicycle, discrete] = unicycle_multiagent_scenario6(deltaT, x, ctrl_multiplier, gradDensityHandles, c1, c2, c3, c4, c5, c6, p, single_int_p, agent_number, dens_bool)
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
    if nargin < 14
        dens_bool = true;
    end

    % Parameters
    rad_from_goal = p.rad_from_goal;
    saturation_vel = 1;
    saturation_omega = 10;
    Kp = 100;
    theta = x(3);
    isgoal = false;
    unicycle = true;
    discrete = false;
    
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
        case 5
            xd = p.xd5;
            grad_density = gradDensityHandles.grad_density5;
        case 6
            xd = p.xd6;
            grad_density = gradDensityHandles.grad_density6;
        otherwise
            error('Invalid agent_number');
    end

    % Control logic based on the density function boolean
    if dens_bool
        % Switch control if near the goal
        if norm(x - xd) < rad_from_goal
            u_hat = -single_int_p.K*(x(1:2)-xd(1:2));
            % u_hat = [0;0];
            isgoal = true;
        else
            u_hat = ctrl_multiplier * grad_density(x, c1, c2, c3, c4, c5, c6);
        end
    else
        disp('-----------------using navigation functions!!--------------------')
        u_hat = -ctrl_multiplier * grad_phi_f(x);
        
    end
    vel = norm(u_hat);
    theta_tilda = atan2(u_hat(2), u_hat(1));

    % Compute state derivatives
    x1_dot = vel * cos(theta);
    x2_dot = vel * sin(theta);
    
    % Use backstepping to map single integrator control to unicycle model
    theta_tilda_dot = backwardEuler(agent_number, theta_tilda, deltaT);
    w = theta_tilda_dot - Kp * (theta - theta_tilda);
    theta_dot = w;

    % Control input
    u = [vel; w];
    x_dot = [x1_dot; x2_dot; theta_dot];

    % Saturation
    if abs(u(1)) > saturation_vel
        % disp(['----- Agent ', num2str(agent_number), ' reached vel limits'])
        u(1) = saturation_vel * sign(u(1));
    end
    if abs(u(2)) > saturation_omega
        % disp(['----- Agent ', num2str(agent_number), ' reached w limits'])
        u(2) = saturation_omega * sign(u(2));
    end
    % disp(['----- Agent ', num2str(agent_number), ' vel:' ,num2str(u(1))])
end
