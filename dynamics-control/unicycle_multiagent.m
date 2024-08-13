function [x_dot, u] = unicycle_multiagent(deltaT,x,ctrl_multiplier, gradDensityHandles,c1,c2,c3,c4, p, single_int_p, agent_number, dens_bool)
%singleIntegrator
%   Propagates the kinematic model (i.e. single integrator system) with a
%   control and changes to a LQR control given a radius from goal
% Inputs:
%   t                   : Discrete time
%   x                   : Current state of the system (R^n)
%   ctrl_multiplier     : Scaling value to increase control value input
%   p                   : Navigation parameter containing xd (Goal state)
%                           and radius from goal rad_from_goal (R^1)
%   single_int_p        : Single Integrator parameters. Specifically the
%                           dynamical system matrix A, B and FB optimal
%                           gain K from the Riccatti equation
%   dens_bool           : Boolean to enable density functions vs navigation
%                           functions
% Outputs
%   x_dot               : The velocity state or control input (x_dot = u)
%   u                   : Control input
% TODO(AZ): Need to think about how to generalize this formulation... right
% now it seems okay to have different dynamics but backstepping
% formulation needs to be more general

%% default args
if nargin < 12
    dens_bool = true;
end
rad_from_goal = p.rad_from_goal;
saturation = 2;
Kp = 10;
theta = x(3);

%% control for each agent
if agent_number == 1
    xd = p.xd1;
    if dens_bool
        u_hat = ctrl_multiplier*gradDensityHandles.grad_density1(x,c1,c2,c3,c4);
        vel = norm(u_hat);
        theta_tilda = atan2(u_hat(2),u_hat(1));
    else
        u_hat = -ctrl_multiplier*grad_phi_f(x);
    end
end

if agent_number == 2
    xd = p.xd2;
    if dens_bool
        u_hat = ctrl_multiplier*gradDensityHandles.grad_density2(x,c1,c2,c3,c4);
        vel = norm(u_hat);
        theta_tilda = atan2(u_hat(2),u_hat(1));
    else
        u_hat = -ctrl_multiplier*grad_phi_f(x);
    end
end

if agent_number == 3
    xd = p.xd3;
    if dens_bool
        u_hat = ctrl_multiplier*gradDensityHandles.grad_density3(x,c1,c2,c3,c4);
        vel = norm(u_hat);
        theta_tilda = atan2(u_hat(2),u_hat(1));
    
    else
        u_hat = -ctrl_multiplier*grad_phi_f(x);
    end
end

if agent_number == 4
    xd = p.xd4;
    if dens_bool
        u_hat = ctrl_multiplier*gradDensityHandles.grad_density4(x,c1,c2,c3,c4);
        vel = norm(u_hat);
        theta_tilda = atan2(u_hat(2),u_hat(1));
    else
        u_hat = -ctrl_multiplier*grad_phi_f(x);
    end
end
    
%% add wrapping for theta tilde
if theta_tilda > pi
    j = 1;
    while theta_tilda/pi > (2*j) + 1
        j = j+1;
    end
    theta_tilda = theta_tilda - j*(2*pi);
end
if theta_tilda < -pi
    j = 1;
    while theta_tilda/(-pi) > (2*j) + 1
        j = j+1;
    end
    theta_tilda = theta_tilda + j*(2*pi);
end

%% Switch control
if(norm(x-xd)<rad_from_goal)
    x_dot = zeros(length(x), 1);
    u = zeros(2, 1);

    % LQR Feedback Gain
    % u_hat = -single_int_p.K*(x-xd);
    % x_dot = single_int_p.A*x+single_int_p.B*u_hat;
else
    % stack state and controls
    x1_dot = vel*cos(theta);
    x2_dot = vel*sin(theta);
    
    % use backstepping to map single integrator control to unicycle model
    theta_tilda_dot = backwardEuler(theta_tilda,deltaT);
    w = theta_tilda_dot - Kp*(theta-theta_tilda);
    theta_dot = w;
    u = [vel; w];
    x_dot = [x1_dot; x2_dot; theta_dot];
end

%% add saturation
[max_u, ~] = max(abs(u(1)));
if max_u >= saturation
    u(1) = u(1)/max_u*saturation;
end

