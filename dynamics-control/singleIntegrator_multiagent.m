function [x_dot, u, isgoal, unicycle, discrete] = singleIntegrator_multiagent(t,x,ctrl_multiplier, gradDensityHandles,c1,c2,c3,c4, p, single_int_p, agent_number, dens_bool)
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

if nargin < 12
    dens_bool = true;
end
isgoal = false;
unicycle = false;
discrete = false;

if agent_number == 1
rad_from_goal = p.rad_from_goal;
xd = p.xd1;
saturation = 2;

if dens_bool
    u = ctrl_multiplier*gradDensityHandles.grad_density1(x,c1,c2,c3,c4);
    % u = ctrl_multiplier*grad_density_f1(x);
else
    u = -ctrl_multiplier*grad_phi_f(x);
end
end

if agent_number == 2
rad_from_goal = p.rad_from_goal;
xd = p.xd2;
saturation = 2;

if dens_bool
    u = ctrl_multiplier*gradDensityHandles.grad_density2(x,c1,c2,c3,c4);
    % u = ctrl_multiplier*grad_density_f2(x);
else
    u = -ctrl_multiplier*grad_phi_f(x);
end
end

if agent_number == 3
rad_from_goal = p.rad_from_goal;
xd = p.xd3;
saturation = 2;

if dens_bool
    u = ctrl_multiplier*gradDensityHandles.grad_density3(x,c1,c2,c3,c4);
    % u = ctrl_multiplier*grad_density_f3(x);
else
    u = -ctrl_multiplier*grad_phi_f(x);
end
end

if agent_number == 4
rad_from_goal = p.rad_from_goal;
xd = p.xd4;
saturation = 2;

if dens_bool
    u = ctrl_multiplier*gradDensityHandles.grad_density4(x,c1,c2,c3,c4);
    % u = ctrl_multiplier*grad_density_f4(x);
else
    u = -ctrl_multiplier*grad_phi_f(x);
end
end


[max_u, ~] = max(abs(u));
if max_u >= saturation
    u = u/max_u*saturation;
end
    

% Switch control

if(norm(x-xd)<rad_from_goal)
    isgoal=true;
    %x_dot = zeros(length(x), 1);
    
    % LQR Feedback Gain
    u = -single_int_p.K*(x-xd);
    x_dot = single_int_p.A*x+single_int_p.B*u;
else
    x_dot = u; % Since A = 0 -> x_dot = A*x + u -> x_dot = u
end

end

