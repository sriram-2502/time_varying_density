function [x_dot] = singleIntegrator(t,x,navigation_params)% xdot = u dynamics

x_goal = navigation_params.x_goal;
rad_from_goal = navigation_params.rad_from_goal;
saturation = navigation_params.saturation;

[~,~,~,lqr_params] = get_params(x);
A = lqr_params.A; B = lqr_params.B;
K = lqr_params.K;

% x1 = x(1,:); 
% x2 = x(2,:);

%% update x_dot;
% if(norm(x-x_goal')<rad_from_goal)
%     x_dot = zeros(length(x), 1);
%     % LQR Feedback Gain (has overshoot issues)
%     %x_dot = A*x + B*K*(x-x_goal');
% else
%     % Preserve the gradient of the control
%     % Normalization method to bound if constraint not satisfied while
%     % preserving the direction of the gradient vector
%     max_u = max(abs(u));
%     if max_u <= saturation % If satisfy friction constraints
%         x_dot = u;
%     else
%         x_dot = u/max_u*saturation;
% %    x_dot = 2*tanh(u);
% %    x_dot = u;
%     end
% end

% Parameters
rad_from_goal = navigation_params.rad_from_goal;
xd = navigation_params.x_goal;
lqr_enable = false;
normalization = true;
stop_enable = true;
u_max = navigation_params.saturation;

% u = ctrl_multiplier*grad_density_f(x,t);
% u = u(1:length(x)); % Take out time term
% u = ctrl_multiplier*grad_density_f(x,t);
u = navigation_params.ctrl_multiplier*grad_density_f(x,t);
% u = u(1:length(x)) + x_dot_d_f(t)';
% u = u(1:length(x)); + jac_xd_f(t); % gradient of rho wrt x + Time derivative of reference traj
u = u(1:length(x));

if strcmp(class(xd), 'sym')
    % fprintf("symbolic\n");
    xd = xd_f(t)';
end

if normalization
    % fprintf("Saturation\n");
    [max_u, ~] = max(abs(u));
    if max_u >= u_max
       u = u(1:length(x))/max_u*u_max;
    else
        u = u(1:length(x));
    end
        
end

if(norm(x-xd)<rad_from_goal)
    %fprintf("Near goal\n");
    if stop_enable
         x_dot = zeros(length(x), 1);
    elseif ~lqr_enable
        x_dot = u;
    elseif lqr_enable
        u = -K*(x-xd);
        x_dot = u;
    end
else
    x_dot = u(1:length(x)); % Take gradient of only states x
end


end

