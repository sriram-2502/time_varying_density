function [x_dot, u, isgoal, unicycle, discrete] = doubleIntegrator_multiagent(t,x,ctrl_multiplier,gradDensityHandles,c1,c2,c3,c4, p, dbl_int_p,agent_number, dens_bool)

%singleIntegrator
%   Propagates the dynamic model (i.e. double integrator system) with a
%   control and changes to a LQR control given a radius from goal.
%   Assumes canonical state space form
% Inputs:
%   t                   : Discrete time
%   x                   : Current state of the system (R^(nx1))
%   ctrl_multiplier     : Scaling value to increase control value input
%   p                   : Navigation parameter containing xd (Goal state)
%                           and radius from goal rad_from_goal (R^1) for
%                           switch control
%   dbl_int_p           : Double Integrator parameters. Specifically the
%                           dynamical system matrix A, B and FB optimal
%                           gain K from the Riccatti equation
%   dens_bool           : Boolean to enable density functions vs navigation
%                           functions
% Outputs
%   x_dot               : The derivative states (R^(nx1))
%   u                   : Control input (R^(mx1))

% TODO(AZ): Need to think about how to generalize this formulation right

% Parameters
isgoal = false;
unicycle = false;
discrete = true;

rad_from_goal = p.rad_from_goal;
kp = p.kp;
kd = p.kd;
dim_x = size(dbl_int_p.A,1);
dim_u = dbl_int_p.u_dim;


% ** Assumes that # of spatial states and time derivative states are equal **
spatial_x = x(1:dim_x/2);
spatial_x_dim = dim_x/2;

if nargin < 12
    dens_bool = true;
end


if agent_number == 1
    rad_from_goal = p.rad_from_goal;
    xd = p.xd1;
    saturation = 1;
    
    if dens_bool
        grad_density_vals = ctrl_multiplier*gradDensityHandles.grad_density1(x(1:2),c1(1:2),c2(1:2),c3(1:2),c4(1:2));
        u = kp*grad_density_vals - kd*x(dim_x - dim_u + 1:dim_x);
        % disp([u,kp*grad_density_vals, -kd*x(dim_x - dim_u + 1:dim_x)])
    else
        grad_density_vals = -ctrl_multiplier*grad_phi_f(x);
        u = kp*grad_density_vals - kd*x(dim_x - dim_u + 1:dim_x);
    end
end

if agent_number == 2
    rad_from_goal = p.rad_from_goal;
    xd = p.xd2;
    saturation = 2;
    
    if dens_bool
        grad_density_vals = ctrl_multiplier*gradDensityHandles.grad_density2(x(1:2),c1(1:2),c2(1:2),c3(1:2),c4(1:2));
        u = kp*grad_density_vals - kd*x(dim_x - dim_u + 1:dim_x);
        % disp([u,kp*grad_density_vals, -kd*x(dim_x - dim_u + 1:dim_x)])
    else
        grad_density_vals = -ctrl_multiplier*grad_phi_f(x);
        u = kp*grad_density_vals - kd*x(dim_x - dim_u + 1:dim_x);
    end
end

if agent_number == 3
    rad_from_goal = p.rad_from_goal;
    xd = p.xd3;
    saturation = 2;
    
    if dens_bool
        grad_density_vals = ctrl_multiplier*gradDensityHandles.grad_density3(x(1:2),c1(1:2),c2(1:2),c3(1:2),c4(1:2));
        u = kp*grad_density_vals - kd*x(dim_x - dim_u + 1:dim_x);
        % disp([u,kp*grad_density_vals, -kd*x(dim_x - dim_u + 1:dim_x)])
    else
        grad_density_vals = -ctrl_multiplier*grad_phi_f(x);
        u = kp*grad_density_vals - kd*x(dim_x - dim_u + 1:dim_x);
    end
end

if agent_number == 4
    rad_from_goal = p.rad_from_goal;
    xd = p.xd4;
    saturation = 2;
    
    if dens_bool
        grad_density_vals = ctrl_multiplier*gradDensityHandles.grad_density4(x(1:2),c1(1:2),c2(1:2),c3(1:2),c4(1:2));
        u = kp*grad_density_vals - kd*x(dim_x - dim_u + 1:dim_x);
        % disp([u,kp*grad_density_vals, -kd*x(dim_x - dim_u + 1:dim_x)])
    else
        grad_density_vals = -ctrl_multiplier*grad_phi_f(x);
        u = kp*grad_density_vals - kd*x(dim_x - dim_u + 1:dim_x);
    end
end

% Saturation
[max_u, ~] = max(abs(u));
if max_u >= saturation
    u = u/max_u*saturation;  
end  

% Switch control
% TODO: LQR OSCILLATING MAYBE BC DISCRETIZATION
if(norm(x(1:spatial_x_dim)-xd(1:spatial_x_dim))<rad_from_goal)
    isgoal=true;
    disp(['----- Agent ', num2str(agent_number), ' LQR active -----'])
    % LQR Feedback Gain
    u = -dbl_int_p.K_d*(x-xd);
end

x_dot = dbl_int_p.A_d*x + dbl_int_p.B_d*u;

end