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

[x_dot, u_euler, isgoal] = dynamics(deltaT, x_temp,ctrl_multiplier, gradDensityHandles,c1,c2,c3,c4, p, dyn_p, agent_number, dens_bool);

if(~isgoal)
    x_euler = x_temp + deltaT*x_dot;
    % wrap theta
    theta = x_euler(3);
    if theta > pi
        disp('wrap')
        j = 1;
        while theta/pi > (2*j) + 1
            j = j+1;
        end
        theta = theta - j*(2*pi);
    end
    if theta < -pi
        disp('wrap')
        j = 1;
        while theta/(-pi) > (2*j) + 1
            j = j+1;
        end
        theta = theta + j*(2*pi);
    end
    x_euler(3) = theta;
else
    disp(['----- ', num2str(agent_number), ' reached goal -----'])
    x_euler = x_temp;
    x_dot = zeros(size(x_temp));
    u_euler = zeros(2,1);
end

% transpose and return variables
x_euler = x_euler';
x_dot = x_dot';
u_euler = u_euler';

end