%% Obstacle Avoidance using single integrator dynamics
clc; clear; close all
% add paths
mkdir('animations');
addpath('./functions');
addpath('./utils');
addpath('dynamics-control');
addpath('./bump_lib');

colors = colororder;
blue = colors(1,:);
red = colors(2,:);
yellow = colors(3,:);
purple = colors(4,:);
green = colors(5,:);

%% Problem Setup

% Environment size
num_agents = 3;

% Start and goal positions for agents
nav_p.x01 = [5; 2; 0];               % Start position for agent 1
nav_p.xd1 = [25;12.5;0];      % Goal position for agent 1

nav_p.x02 = [5; 17.5;0];           % Start position for agent 2
nav_p.xd2 = [0;25;0];      % Goal position for agent 2

nav_p.x03 = [20; 10;-pi];             % Start position for agent 3
nav_p.xd3 = [0;0;0];      % Goal position for agent 3
            
% Obstacle parameters
nav_p.p = 2; % p-norm for obstacle set
nav_p.rad_from_goal = 4; % Radius for stopping density feedback control

% Density function parameters
% nav_p.r11 = 1; nav_p.r12 = 3;
% nav_p.r21 = 1; nav_p.r22 = 3;
% nav_p.r31 = 0.5; nav_p.r32 = 2;
% nav_p.r41 = 0.5; nav_p.r42 = 2;

% Density function parameters
% self raidus is 0.75 so enlareged radius is 1.5
nav_p.r11 = 5; nav_p.r12 = 8;
nav_p.r21 = 5; nav_p.r22 = 8;
nav_p.r31 = 5; nav_p.r32 = 8;


% Obstacle centers
nav_p.c0 = 0; % Weaker condition for unsafe set

% Transformation parameters
theta = 0; % Rotation angle in radians
stretch = [1; 1]; % Stretch factors
gamma = 0; % Post-rotation angle in radians

% Density function exponent
nav_p.alpha = 0.2; % Recommended value: 0.2

% Simulation parameters
M = 5000; % Number of loop iterations
deltaT = 0.01; % Time step
ctrl_multiplier = 100; % Control parameter

% Optimization settings
vpa_enable = true;
if vpa_enable
    optimize = false; % Set to false for faster symbolic tool generation
end

% Density Function Formulation
syms x [2,1] real
[A, A_inv] = transformationMatrix(theta, stretch, 2);


%% Form System Matrices and LQR Gain
[single_int_p, dbl_int_p] = generateStateSpace(deltaT, x);

% Initialize state and control arrays
x_euler = zeros(M, size(nav_p.x01, 1), num_agents);
x_euler(1,1:size(nav_p.x01, 1),1) = nav_p.x01;
x_euler(1,1:size(nav_p.x02, 1),2) = nav_p.x02;
x_euler(1,1:size(nav_p.x03, 1),3) = nav_p.x03;
u_euler = zeros(M, 2, num_agents);

% Initialize temporary state variables
x_temp = {nav_p.x01, nav_p.x02, nav_p.x03};
c = x_temp; % Use 'c' to hold current state

% Create function handles for bump functions and gradients
bumpHandles = createBumpHandles();
gradDensityHandles = createGradientHandles(nav_p);

%% Simulation loop
% clear persistant values
clear backwardEuler
clear forwardEuler

tic
for iter = 2:M
    if mod(iter, 1) == 0
        % disp(['iter:', num2str(iter)]);
    end

    % Update states and controls for each vehicle
    for i = 1:num_agents
        [x_euler(iter, :, i), u_euler(iter, :, i)] = forwardEuler_multiagent_scenario3(nav_p, deltaT, ctrl_multiplier, ...
            @unicycle_multiagent_scenario3, gradDensityHandles, c{1}, c{2}, c{3}, single_int_p, x_temp{i}, i);
        x_temp{i} = x_euler(iter, :, i)'; % Update temporary state
        c{i} = x_temp{i}; % Update control state
    end
end
toc
%% Plot Time Domain
fig_titles = {'States', 'Velocity', 'Omega'};

% Plot states
figure();
for i = 1:num_agents
    subplot(2, 2, i);
    plot(1:M, squeeze(x_euler(:, 1:2, i)),'-', 'LineWidth', 2);
    ylabel('positions');
    xlabel('timesteps');
    title(sprintf('Agent %d - positions', i));
    box on;
end

% plot angles
figure();
for i = 1:num_agents
    subplot(2, 2, i);
    plot(1:M, squeeze(x_euler(:, 3, i)),'-', 'LineWidth', 2);
    ylabel('angles');
    xlabel('timesteps');
    title(sprintf('Agent %d - angles', i));
    box on;
end

% Plot velocities
figure();
for i = 1:num_agents
    subplot(2, 2, i);
    plot(1:M, squeeze(u_euler(:, 1, i)), 'LineWidth', 2);
    ylabel('velocity');
    xlabel('timesteps');
    title(sprintf('Agent %d - Velocity', i));
    box on;
end

% Plot omegas
figure();
for i = 1:num_agents
    subplot(2, 2, i);
    plot(1:M, squeeze(u_euler(:, 2, i)), 'LineWidth', 2);
    ylabel('omega');
    xlabel('timesteps');
    title(sprintf('Agent %d - Omega', i));
    box on;
end

%%
% Define colors using colororder
colors = colororder;
blue = colors(1, :);
red = colors(2, :);
yellow = colors(3, :);
purple = colors(4, :);

% Assign these colors to your traces and plot elements
color_map = {blue, red, yellow};

% Define the time steps for the snapshots and their corresponding trail lengths
snapshot_times = [1, 5000, 7000, 12500]; % Time steps for snapshots
trail_lengths = [2000, 2000, 2000, 6000]; % Trail lengths corresponding to each snapshot

% Define the figure and subplot layout
figure;
subplot_positions = [1, 2, 5, 6]; % Subplot positions for 2x4 grid

% Loop over the snapshots
for idx = 1:length(snapshot_times)
    jj = snapshot_times(idx); % Current time step for the snapshot
    trace_length = trail_lengths(idx); % Get the trail length for the current snapshot
    
    % Define the subplot position
    subplot(2, 4, subplot_positions(idx)); % Create subplot
    
    % Ensure the figure is cleared for the new plot
    hold off;
    cla;
    hold on;
    box on;

    % Plot desired positions
    for i = 1:num_agents
        % Plot the start position as a solid marker
        plot(nav_p.(['x0' num2str(i)])(1), nav_p.(['x0' num2str(i)])(2), ...
         'o', 'MarkerSize', 10, 'MarkerFaceColor', color_map{i}, 'MarkerEdgeColor', color_map{i});

        % Plot the goal position as a dashed circle
        theta_fill = linspace(0, 2*pi, 100);
        x_fill = 1 * nav_p.(['r' num2str(i) '1']) * cos(theta_fill) + nav_p.(['xd' num2str(i)])(1);
        y_fill = 1 * nav_p.(['r' num2str(i) '1']) * sin(theta_fill) + nav_p.(['xd' num2str(i)])(2);
        plot(x_fill, y_fill, '-', 'Color', color_map{i}, 'LineWidth', 2);
    end

    % Update and plot traces for each vehicle
    for i = 1:num_agents
        % Ensure we are within bounds
        if jj <= size(x_euler, 1)
            % Update trace for the current snapshot
            new_position = squeeze(x_euler(jj, 1:2, i));
            % Get the historical positions up to the current snapshot
            trace_start = max(1, jj-trace_length+1);
            historical_positions = squeeze(x_euler(trace_start:jj, 1:2, i));

            % Plot traces
            plot(historical_positions(:, 1), historical_positions(:, 2), '--', 'Color', color_map{i}, 'LineWidth', 2);

            % Plot the current position with the heading direction
            plotCircleAndHeading(new_position(1), ...
                                 new_position(2), ...
                                 squeeze(x_euler(jj, 3, i)), ...
                                 nav_p.(['r' num2str(i) '1']), ...
                                 nav_p.(['r' num2str(i) '2']), ...
                                 color_map{i});
        end
    end

    % Set plot properties
    xlim([-10, 32]);
    ylim([-10, 32]);
    % axis square;
    set(gca, 'LineWidth', 2, 'FontSize', 20); % Box line width and font size

    %  % Add x-ticks for bottom two snapshots
    % if idx == 3 || idx == 4
    %     set(gca, 'XTick', linspace(-env_size, env_size, 3)); % Customize x-ticks as needed
    % else
    %     set(gca, 'XTick', []); % Remove x-ticks for other snapshots
    % end
    
    % % Add y-ticks for left two snapshots
    % if idx == 1 || idx == 3
    %     set(gca, 'YTick', linspace(-env_size, env_size, 3)); % Customize y-ticks as needed
    % else
    %     set(gca, 'YTick', []); % Remove y-ticks for other snapshots
    % end

    % Add dummy subplot for legend
    subplot(2,4,3)
    % Dummy plot entries for the legend
    plot(nan, nan, 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k'); hold on
    plot(nan, nan, 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'none', 'MarkerEdgeColor', 'k', 'LineStyle', 'none', 'LineWidth',2); hold on
    plot(nan, nan, '--', 'Color', 'k', 'LineWidth', 2); % Dummy trace

    legend('Start', 'Goal', 'Trajectory', 'Location', 'best');

end

% Adjust layout for better spacing
set(gcf, 'Position', [100, 100, 1200, 600]);

%% Animation Setup
skip_rate = 10;
save_videos = true;
trace_length = 100; % Number of recent positions to keep in the trace

% Define the video file path and name
video_file_path = 'animations/scenario.mp4'; % Adjust path if necessary

if save_videos
    vidFile = VideoWriter(video_file_path, 'MPEG-4');
    vidFile.FrameRate = 5;
    open(vidFile);
end

% Initialize storage for traces
traces = cell(num_agents, 1);
for i = 1:num_agents
    traces{i} = squeeze(x_euler(1, 1:2, i)); % Initialize with the first position
end

% Define colors using colororder
colors = colororder;
blue = colors(1, :);
red = colors(2, :);
yellow = colors(3, :);
purple = colors(4, :);

% Assign these colors to your traces and plot elements
color_map = {blue, red, yellow, purple};

% Animation loop
for jj = 1:skip_rate:M
    % Create figure and set properties
    f1 = figure(999);
    clf(f1);
    set(f1, 'Color', 'white');
    hold on;
    
    % Plot desired positions
    for i = 1:num_agents
        plot(nav_p.(['xd' num2str(i)])(1), nav_p.(['xd' num2str(i)])(2), ...
             'o', 'MarkerSize', 10, 'MarkerFaceColor', color_map{i}, 'Color', color_map{i});
    end

    % Update and plot traces for each vehicle
    for i = 1:num_agents
        % Update trace
        new_position = squeeze(x_euler(jj, 1:2, i)); % Ensure new_position is a row vector
        
        % Check dimension consistency
        if size(new_position, 2) == 2
            traces{i} = [traces{i}; new_position];
            
            % Limit trace length
            if size(traces{i}, 1) > trace_length
                traces{i} = traces{i}(end - trace_length + 1:end, :); % Keep only the most recent positions
            end
            
            % Plot circle and heading
            plotCircleAndHeading(squeeze(x_euler(jj, 1, i)), ...
                                 squeeze(x_euler(jj, 2, i)), ...
                                 squeeze(x_euler(jj, 3, i)), ...
                                 nav_p.(['r' num2str(i) '1'])/2, ...
                                 nav_p.(['r' num2str(i) '2'])/2, ...
                                 color_map{i});
                             
            % Plot trace
            plot(traces{i}(:, 1), traces{i}(:, 2), '--', 'Color', color_map{i}, 'LineWidth', 2);
        else
            warning('new_position dimensions do not match the expected size.');
        end
    end

    % Set plot properties
    box on;
    set(gca, 'XTick', [], 'YTick', [], 'LineWidth', 2, 'FontSize', 20); % Box line width and font size
    axis square
    axis tight;
    xlim([-10, 32]);
    ylim([-10, 32]);
    
    % Add timestamp
    timestamp = sprintf('Time: %0.2f s', deltaT * jj);
    text(0.3, 0.1, timestamp, 'Units', 'normalized', 'FontSize', 12);
    
    % Write frame to video if saving
    if save_videos
        frame = getframe(gcf);
        writeVideo(vidFile, frame);
    end
end

% Close video file if saving
if save_videos
    close(vidFile);
end


%% Function to Plot Circle, Heading Line, and Sensing Region
function plotCircleAndHeading(x, y, theta, r1, r2, color)
    % Plot the sensing region as a filled circle with transparency
    theta_fill = linspace(0, 2*pi, 100);
    x_fill = r2 * cos(theta_fill) + x;
    y_fill = r2 * sin(theta_fill) + y;
    fill(x_fill, y_fill, color, 'FaceAlpha', 0.3, 'EdgeColor', 'none');

    % Plot the robot's circle as a filled circle with transparency
    theta_robot = linspace(0, 2*pi, 100);
    x_robot = r1 * cos(theta_robot) + x;
    y_robot = r1 * sin(theta_robot) + y;
    fill(x_robot, y_robot, color, 'FaceAlpha', 0.6, 'EdgeColor', 'black', 'LineWidth',2);

    % Compute the end point of the heading line
    line_end_x = x + 1.5 * r1 * cos(theta);
    line_end_y = y + 1.5 * r1 * sin(theta);

    % Plot the heading line
    plot([x, line_end_x], [y, line_end_y], 'Color', 'black', 'LineWidth', 2);
end


%% function to create bump function handles
function bumpHandles = createBumpHandles()
    % Define the function handles for each bump
    syms x [2,1] real
    bump1 = @(nav_p,c1,c2,c3) formPNormBump(nav_p.r21, nav_p.r22, c2, x, nav_p.p, true) ...
                                  .* formPNormBump(nav_p.r31, nav_p.r32, c3, x, nav_p.p, true);
    
    bump2 = @(nav_p,c1,c2,c3) formPNormBump(nav_p.r11, nav_p.r12, c1, x, nav_p.p, true) ...
                                  .* formPNormBump(nav_p.r31, nav_p.r32, c3, x, nav_p.p, true);
    
    bump3 = @(nav_p,c1,c2,c3) formPNormBump(nav_p.r11, nav_p.r12, c1, x, nav_p.p, true) ...
                                  .* formPNormBump(nav_p.r21, nav_p.r22, c2, x, nav_p.p, true);

    

    bumpHandles.bump1Handle = bump1;
    bumpHandles.bump2Handle = bump2;
    bumpHandles.bump3Handle = bump3;
end

%% function to create gradient of density function handles
function gradDensityHandles = createGradientHandles(nav_p)
    bumpHandles = createBumpHandles();
    syms x [2,1] real
    syms c1 [2,1] real
    syms c2 [2,1] real
    syms c3 [2,1] real

    % form V(x) for each agent
    g1 = 1/norm(x(1:2)-nav_p.xd1(1:2))^(2*nav_p.alpha); 
    g2 = 1/norm(x(1:2)-nav_p.xd2(1:2))^(2*nav_p.alpha); 
    g3 = 1/norm(x(1:2)-nav_p.xd3(1:2))^(2*nav_p.alpha);

    % Compute densities using bump handles
    grad_density1 = gradient(g1 * bumpHandles.bump1Handle(nav_p,c1,c2,c3),x);
    grad_density2 = gradient(g2 * bumpHandles.bump2Handle(nav_p,c1,c2,c3),x);
    grad_density3 = gradient(g3 * bumpHandles.bump3Handle(nav_p,c1,c2,c3),x);

    % Create function handles
    optimize = false;
    grad_density1_sym = matlabFunction(grad_density1, 'File', 'functions/grad_density_f1', 'Vars', {x,c1,c2,c3}, 'Optimize', optimize);
    grad_density2_sym = matlabFunction(grad_density2, 'File', 'functions/grad_density_f2', 'Vars', {x,c1,c2,c3}, 'Optimize', optimize);
    grad_density3_sym = matlabFunction(grad_density3, 'File', 'functions/grad_density_f3', 'Vars', {x,c1,c2,c3}, 'Optimize', optimize);

    % Compute gradients of densities with respect to x
    gradDensityHandles.grad_density1 = @(x,c1,c2,c3) grad_density1_sym(x,c1,c2,c3);
    gradDensityHandles.grad_density2 = @(x,c1,c2,c3) grad_density2_sym(x,c1,c2,c3);
    gradDensityHandles.grad_density3 = @(x,c1,c2,c3) grad_density3_sym(x,c1,c2,c3);
end