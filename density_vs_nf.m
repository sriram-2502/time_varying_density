%% Obstacle Avoidance using single integrator dynamics
clc
clear
close all
clear forwardEuler

colors = colororder;
blue = colors(1,:);
red = colors(2,:);
yellow = colors(3,:);
green = colors(5,:);
obs_color = [.7 .7 .7]; % Obstacle color -> Grey

%% load NF data
nf_log = readtable('nf_log.csv');
nf_log_x = nf_log.('RobotX');
nf_log_y = nf_log.('RobotY');

%% Problem setup
mkdir('animations');
mkdir('functions');
mkdir('utils')
mkdir('dynamics-control')
addpath('./functions');
addpath('./utils');
addpath('dynamics-control');
addpath('bump_lib');
tic

%% Parameter setup
% Parameters
% Start position
syms t
nav_p.x0 = [nf_log_x(1);nf_log_y(1)]; % Start position

time_varying_goal = true;
target_decrement=0.06;
if time_varying_goal
    nav_p.xd = [-0.8+target_decrement*t;1.2-target_decrement*t];
else
    nav_p.xd = [3;-1.5]; % Final desired goal position | Keep 0 until ensured backstepping works with xd
end

% Target set radius | Radius when to stop using Density FB control
nav_p.rad_from_goal = 0.5;

% Function: f(x):= k/exp(a/(1-||x-c||^2)) - k/e^a
nav_p.a = 0.1; % Positive definite | Best value 0.1
nav_p.p = 4;

diffeomorphism = false;

if ~diffeomorphism
    A = eye(2);
    A_inv = eye(2);
end

% Obstancle centers (e.g. c1 and c2) | Domain R^n
time_varying_obs = false;
num_obs = 3;
nav_p.c1 = [-0.25;-0.2]; nav_p.r11 = 0.3; nav_p.r12 = nav_p.r11+0.2;
nav_p.c2 = [0.5;0.8]; nav_p.r21 = 0.4; nav_p.r22 = nav_p.r21+0.2;
nav_p.c3 = [0.7;0.7]; nav_p.r31 = 0.4; nav_p.r32 = nav_p.r31+0.2;

% Function: g(x):= 1/||x||^alpha
nav_p.alpha = 0.2; % Best value 0.2

% Discrete time parameters
x_euler = nav_p.x0;
N = 4000; %timesteps
deltaT = 0.1;
gamma = 0.25; % ctrl mult | Good value: 2 | Bad: 1e-10
normalization = 2; % Enforce control constraints

% Backstepping enable
plot_single_integrator = true; % single integrator

% Plot graph
plot_graph = true;

% Optimize function handle generation
optimize = true; % Decision to optimize function generation | false for faster generation time
vpa_enable = true; % Enable variable precision arithmetic

if vpa_enable
    optimize = false; % Set optimize also false s.t. symbolic tool generation is fast / manageable
end

% Graph gradients
graph_misc_plots = false; % Also enables gradient gyf

% mp4 parameters
obs_avoid_file_name = sprintf(...
    'animations/lane_tracking_ub_constr_exp_ref_tracking',...
    deltaT, gamma);
save_videos = true;
local_view = false;

grad_file_name = 'animations/flow_field_time_varying_obs_10tracking.mp4';
save_grad_videos = false;



%% Create function handles
% Create function of density and bump function
syms x [2,1] real

% Obstacle function
bump = formPNormBump(nav_p.r11,nav_p.r12,nav_p.c1, x, nav_p.p, true)*formPNormBump(nav_p.r21, nav_p.r22, nav_p.c2, x, nav_p.p, true)*...
    formPNormBump(nav_p.r31, nav_p.r32, nav_p.c3, x, nav_p.p, true);

g = 1/norm(x-nav_p.xd)^(2*nav_p.alpha); % rho

% Density function
density = g*bump;

grad_density = gradient(density, [x;t]);

% Create function handles
if time_varying_goal
    matlabFunction(nav_p.xd, 'File', 'functions/xd_f', 'Vars', {t}, 'Optimize', optimize);
    matlabFunction(jacobian(nav_p.xd, t), 'File', 'functions/jac_xd_f', 'Vars', {t}, 'Optimize', optimize);
end
matlabFunction(density, 'File', 'functions/density_f', 'Vars', {x, t}, 'Optimize', optimize);
matlabFunction(grad_density, 'File', 'functions/grad_density_f', 'Vars', {x, t}, 'Optimize', optimize);

%% Form system matrices and LQR Gain
[single_int_p, dbl_int_p] = generateStateSpace(deltaT);


%% using Euler method
[x_euler, u_euler] = forwardEuler_integrator(nav_p, N, deltaT, gamma, @singleIntegrator_t, single_int_p, true, normalization);


%% plot xy traj
figure()
subplot(2,2,1)
% Define the time vector
t = 0:1:30; % Example time range from 0 to 10 seconds

% Calculate the x and y positions of the goal over time
goal_x = -0.8 + target_decrement * t;
goal_y = 1.2 - target_decrement * t;

plot(goal_x, goal_y, '-o', 'Color','black', 'LineWidth',2); hold on;
plot(nf_log_x, nf_log_y,'-o','Color',blue,'LineWidth',2); hold on
plot(x_euler(1:10:330,1),x_euler(1:10:330,2),'-o','Color',red,'LineWidth',2); hold on

% Plot obstacles
n = nav_p.p; % Exponent for the squircle

% Define the kappa parameter for discretizing the squircle
kappa = linspace(0, 2*pi, 100);

% Function to compute squircle points
compute_squircle = @(r1, r2, kappa, n) ...
    [r1 * sign(cos(kappa)) .* abs(cos(kappa)).^(2/n); ...
     r2 * sign(sin(kappa)) .* abs(sin(kappa)).^(2/n)];

% Function to apply rotation matrix
apply_rotation = @(points, angle) ...
    [cos(angle) -sin(angle); sin(angle) cos(angle)] * points;

% Define rotation angle (45 degrees)
rotation_angle = 0;

% Compute squircle points
points1 = compute_squircle(nav_p.r11, nav_p.r11, kappa, n);
points2 = compute_squircle(1.3*nav_p.r21, 0.6667*nav_p.r21, kappa, n);
points3 = compute_squircle(0.6667*nav_p.r31, 1.3*nav_p.r31, kappa, n);

% Apply rotation
points1 = apply_rotation(points1, rotation_angle);
points2 = apply_rotation(points2, rotation_angle);
points3 = apply_rotation(points3, rotation_angle);

% Plot squircle obstacles
% Plot the first squircle obstacle
P1 = polyshape(nav_p.c1(1) + points1(1,:), nav_p.c1(2) + points1(2,:));
plot(P1, 'FaceColor', obs_color, 'LineWidth', 2, 'FaceAlpha', 1.0); hold on;

% Plot the second squircle obstacle
P2 = polyshape(nav_p.c2(1) + points2(1,:), nav_p.c2(2) + points2(2,:));
plot(P2, 'FaceColor', obs_color, 'LineWidth', 2, 'FaceAlpha', 1.0); hold on;

% Plot the third squircle obstacle
P3 = polyshape(nav_p.c3(1) + points3(1,:), nav_p.c3(2) + points3(2,:));
plot(P3, 'FaceColor', obs_color, 'LineWidth', 2, 'FaceAlpha', 1.0); hold on;

xlim([-1,1.5])
ylim([-1,1.5])
legend({'target', 'navigation function', 'density'}, ...
       'Interpreter', 'latex', 'FontSize', 20);

axis square
set(gca, 'Box', 'on', 'LineWidth', 2);
set(gca, 'FontSize', 20);

%% Plot robot with trails
if plot_graph
    % Parameter
    state_index = 1000+1;
    trail_length = 1000+1; %size(x_euler(1:state_index,:),1); % No trail -> trail_length = 1
    tail_grad = exp(linspace(-5,2, trail_length)); % Exponential mapping for gradient
    trail_skip_rate_divisor = 25; % Lower means less trails
    kappa = (0:100-1)*(2*pi/100); % Angles for graphing circle

    obs_1_ind_loc = state_index; % 15500
    obs_2_ind_loc = state_index; % 35000
    obs_3_ind_loc = state_index; % 25000
    obs_4_ind_loc = state_index; % 42000
    
    if trail_length == 1
        trail_skip_rate = 1;
    else
        trail_skip_rate = round(length(tail_grad)/trail_skip_rate_divisor);
    end
    
    figure();
    plot(x_euler(1,1),x_euler(1,2), 'or', 'MarkerSize',10, 'MarkerFaceColor','red'); hold on;
    if time_varying_goal
        xd_t = xd_f(state_index*deltaT);
        plot(xd_t(1), xd_t(2), 'og', 'MarkerSize', 10, 'MarkerFaceColor', 'green'); hold on;
    else
        plot(nav_p.xd(1),nav_p.xd(2),'og', 'MarkerSize',10, 'MarkerFaceColor','green'); hold on;
    end
    dummy_obs = plot(NaN,NaN, 'o','MarkerSize', 10, 'MarkerEdgeColor',...
        'black', 'MarkerFaceColor',obs_color, 'LineWidth', 1.5); % For legend as rectangular object can't be defined as a legend
    
    scatter(x_euler(state_index,1), x_euler(state_index,2), 80, 'filled', 'bo', 'LineWidth', 2); hold on;
    s_plot = scatter(x_euler(state_index-trail_length+1:trail_skip_rate:state_index,1),...
        x_euler(state_index-trail_length+1:trail_skip_rate:state_index,2), 80,'filled','bo', 'LineWidth', 2); hold on;
    s_plot.AlphaData = tail_grad(1:trail_skip_rate:trail_length);
    s_plot.MarkerFaceAlpha = 'flat';
    hold on;
    
    % Define parameters for the squircle
    n = nav_p.p; % Exponent for the squircle
    
    % Define the kappa parameter for discretizing the squircle
    kappa = linspace(0, 2*pi, 100);
    
    % Function to compute squircle points
    compute_squircle = @(r1, r2, kappa, n) ...
        [r1 * sign(cos(kappa)) .* abs(cos(kappa)).^(2/n); ...
         r2 * sign(sin(kappa)) .* abs(sin(kappa)).^(2/n)];
    
    % Function to apply rotation matrix
    apply_rotation = @(points, angle) ...
        [cos(angle) -sin(angle); sin(angle) cos(angle)] * points;
    
    % Define rotation angle (45 degrees)
    rotation_angle = 0;
    
    % Compute squircle points
    points1 = compute_squircle(nav_p.r11, nav_p.r11, kappa, n);
    points2 = compute_squircle(1.3*nav_p.r21, 0.6667*nav_p.r21, kappa, n);
    points3 = compute_squircle(0.6667*nav_p.r31, 1.3*nav_p.r31, kappa, n);
    
    % Apply rotation
    points1 = apply_rotation(points1, rotation_angle);
    points2 = apply_rotation(points2, rotation_angle);
    points3 = apply_rotation(points3, rotation_angle);
    
    % Plot squircle obstacles
    % Plot the first squircle obstacle
    P1 = polyshape(nav_p.c1(1) + points1(1,:), nav_p.c1(2) + points1(2,:));
    plot(P1, 'FaceColor', obs_color, 'LineWidth', 2, 'FaceAlpha', 1.0); hold on;
    
    % Plot the second squircle obstacle
    P2 = polyshape(nav_p.c2(1) + points2(1,:), nav_p.c2(2) + points2(2,:));
    plot(P2, 'FaceColor', obs_color, 'LineWidth', 2, 'FaceAlpha', 1.0); hold on;
    
    % Plot the third squircle obstacle
    P3 = polyshape(nav_p.c3(1) + points3(1,:), nav_p.c3(2) + points3(2,:));
    plot(P3, 'FaceColor', obs_color, 'LineWidth', 2, 'FaceAlpha', 1.0); hold on;
    
    % Set axis properties
    axis equal;
    grid on;
    
    if time_varying_goal
        xd_t = xd_f(0);
        dummy_goal = plot(xd_t(1), xd_t(2), 'MarkerSize',10, 'MarkerFaceColor','green'); hold on;
    else
        dummy_goal = plot(nav_p.xd(1),nav_p.xd(2),'og', 'MarkerSize',10, 'MarkerFaceColor','green'); hold on;
    end
    
    legend('Start','Goal','Obstacle','Trajectory', '','ub constr', '', ...
        '','','','','', ...
        'Location', 'southwest')

    titleName = sprintf('Dyn Obstacles');
    % title(titleName);
    xlabel('$x_1$','interpreter','latex', 'FontSize', 20);
    ylabel('$x_2$','interpreter','latex', 'FontSize', 20);
    xlim([-1.5,1.5]);
    ylim([-1.5,1.5]);
    axis square
    hold on
end

t_span = 1:size(x_euler,1);
figure
plot(t_span,x_euler, 'Linewidth', 2)
xlabel("Iteration")
ylabel("States")
legend("$x_1$", "$x_2$", "Interpreter", "latex")

t_span = 1:size(u_euler,1);
figure
plot(t_span,u_euler, 'Linewidth', 2)
xlabel("Iteration")
ylabel("Control")
legend("$u_1$", "$u_2$", "Interpreter", "latex")

%% Obstacle Avoidance Gif
% Parameter
skip_rate = 50;
obs_color = [.6 .6 .6]; % Obstacle color -> Grey
sensRadColor = [0.9290 0.6940 0.1250]; % Sensing radius color -> Dandelion
kappa = (0:100-1)*(2*pi/100); % Angles for graphing circle
trail_length = 200; % No trail -> trail_length = 1 | Good = 5000
tail_grad = exp(linspace(-10,2, trail_length)); % Exponential mapping for gradient
if trail_length == 1
    trail_skip_rate = 1;
    tail_grad = 1;
else
    trail_skip_rate = round(length(tail_grad)/15); % Trail Skip Rate ~ 0.5-0.1 of trail_length -> good
end

if save_videos
    vidFile = VideoWriter(obs_avoid_file_name, 'MPEG-4');
    vidFile.FrameRate = 5;
    open(vidFile);
end

% jj := frame #
for jj=1:skip_rate:N
    f1 = figure(999);
    clf(f1); % close current figure
    set(f1, 'color', 'white') % Set figure to white
    plot(x_euler(1,1),x_euler(1,2), 'or', 'MarkerSize',10, 'MarkerFaceColor','red'); hold on;
    
    dummy_obs = plot(NaN,NaN, 'o','MarkerSize', 10, 'MarkerEdgeColor',...
        'black', 'MarkerFaceColor',obs_color, 'LineWidth', 1.5); % For legend as rectangular object can't be defined as a legend
    
    
    % Plot obstacles
    n = nav_p.p; % Exponent for the squircle
    
    % Define the kappa parameter for discretizing the squircle
    kappa = linspace(0, 2*pi, 100);
    
    % Function to compute squircle points
    compute_squircle = @(r1, r2, kappa, n) ...
        [r1 * sign(cos(kappa)) .* abs(cos(kappa)).^(2/n); ...
         r2 * sign(sin(kappa)) .* abs(sin(kappa)).^(2/n)];
    
    % Function to apply rotation matrix
    apply_rotation = @(points, angle) ...
        [cos(angle) -sin(angle); sin(angle) cos(angle)] * points;
    
    % Define rotation angle (45 degrees)
    rotation_angle = 0;
    
    % Compute squircle points
    points1 = compute_squircle(nav_p.r11, nav_p.r11, kappa, n);
    points2 = compute_squircle(1.3*nav_p.r21, 0.6667*nav_p.r21, kappa, n);
    points3 = compute_squircle(0.6667*nav_p.r31, 1.3*nav_p.r31, kappa, n);
    
    % Apply rotation
    points1 = apply_rotation(points1, rotation_angle);
    points2 = apply_rotation(points2, rotation_angle);
    points3 = apply_rotation(points3, rotation_angle);
    
    % Plot squircle obstacles
    % Plot the first squircle obstacle
    P1 = polyshape(nav_p.c1(1) + points1(1,:), nav_p.c1(2) + points1(2,:));
    plot(P1, 'FaceColor', obs_color, 'LineWidth', 2, 'FaceAlpha', 1.0); hold on;
    
    % Plot the second squircle obstacle
    P2 = polyshape(nav_p.c2(1) + points2(1,:), nav_p.c2(2) + points2(2,:));
    plot(P2, 'FaceColor', obs_color, 'LineWidth', 2, 'FaceAlpha', 1.0); hold on;
    
    % Plot the third squircle obstacle
    P3 = polyshape(nav_p.c3(1) + points3(1,:), nav_p.c3(2) + points3(2,:));
    plot(P3, 'FaceColor', obs_color, 'LineWidth', 2, 'FaceAlpha', 1.0); hold on;

    
    if time_varying_goal
        xd_t = xd_f(jj*deltaT);
        plot(xd_t(1), xd_t(2), 'og', 'MarkerSize', 10, 'MarkerFaceColor', 'green'); hold on;
    else
        plot(nav_p.xd(1),nav_p.xd(2),'og', 'MarkerSize',10, 'MarkerFaceColor','green'); hold on;
    end
    
    if plot_single_integrator
        if jj < trail_length % Different indexing for when jj < tail_length
            s = scatter(x_euler(1:trail_skip_rate:jj,1),...
                x_euler(1:trail_skip_rate:jj,2), 80,'filled','bo', 'LineWidth', 3); hold on;
            if trail_length > 1
                s.AlphaData = tail_grad(length(tail_grad)-jj+1:trail_skip_rate:end);
                s.MarkerFaceAlpha= 'flat';
            end
        else
            s = scatter(x_euler(jj-trail_length+1:trail_skip_rate:jj,1),...
                x_euler(jj-trail_length+1:trail_skip_rate:jj,2), 80, 'filled','bo', 'LineWidth', 3); hold on;
            if trail_length > 1
                s.AlphaData = tail_grad(1:trail_skip_rate:end);
                s.MarkerFaceAlpha = 'flat';
            end
        end
    end

    legend('Start','Obstacle','ub constr', '',...
        '','','',... % Dummy Legends
        'Goal', 'Ego', 'Location', 'southwest')

    titleName = sprintf('Dyn Obstacles');
    %title(titleName);
    xlabel('$x_1$', 'interpreter', 'latex', 'FontSize', 20)
    ylabel('$x_2$', 'interpreter', 'latex', 'FontSize', 20)
    axis square
    axis tight
    if local_view
        xlim([-3+ xd_t(1),3 + xd_t(1)]);
        ylim([-3,3]);
    else
        xlim([-1.5;1.5]);
        ylim([-1.5;1.5]);
    end
    timestamp = sprintf("Time: %0.2f s", deltaT*jj);
    text(0.6,0.1, timestamp, 'Units', 'normalized');
    if save_videos
        writeVideo(vidFile, getframe(gcf));
    end
end

if save_videos
    close(vidFile);
end
