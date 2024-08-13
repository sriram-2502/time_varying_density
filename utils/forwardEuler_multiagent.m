function [x_euler, u_euler, x_dot] = forwardEuler_multiagent(p, deltaT, ctrl_multiplier, dynamics, gradDensityHandles,c1,c2,c3,c4, dyn_p,x_temp, agent_number, dens_bool)
% TODO(AZ): Create fixed size array instead of using dynamic array
%forwardEuler
%   Forward euler integration to propagate dynamics. Returns x
%
% Inputs:
%   p                   : Parameters specifically containing init
%                           conditions x0 (R^n), goal state xd (R^n) & 
%                           radius from goal (R^1)
%   N                   : Number of forward integration
%   deltaT              : Discrete step size (R^n)
%   ctrl_multiplier     : Multiplier on control (R^n)
%   dynamics            : Function handle to use as dynamics
%   dyn_p               : Dynamic system parameter. More specifically,
%                           the A, B system matrices, the LQR Feedback 
%                           Gain close to goal, and dimension of control
%
% Outputs:
%   x_euler             : Forward integrated states (R^(nx(N+1)))
%   u_euler             : Control input trajectory (R^(nx(N)))
%   x_dot               : State derivative trajectory (R^(nxN))

if nargin <  13
    dens_bool = true;
end


[x_dot, u_euler] = dynamics(deltaT, x_temp,ctrl_multiplier, gradDensityHandles,c1,c2,c3,c4, p, dyn_p, agent_number, dens_bool);

x_euler = x_temp + deltaT*x_dot;
    

x_euler = x_euler';
x_dot = x_dot';
u_euler = u_euler';

end