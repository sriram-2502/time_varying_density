%% Obstacle Avoidance using single integrator dynamics
clc; clear; close all

% add paths
mkdir('animations');
addpath('./functions');
addpath('./utils');
addpath('dynamics-control');
addpath('./bump_lib');

% clear persistant values
clear backwardEuler
clear forwardEuler

%% Problem setup
env_size = 8;
nav_p.x01 = [0;env_size;0]; % Start position agent 1
nav_p.xd1 = [-0.1;-env_size;0]; % Goal position agebt 1
nav_p.x02 = [0;-env_size;0]; % Start position agent 2
nav_p.xd2 = [0.1;env_size;0]; % Goal position agebt 2
nav_p.x03 = [env_size;0;0]; % Start position agent 3
nav_p.xd3 = [-env_size;0.1;0]; % Goal position agebt 3
nav_p.x04 = [-env_size;0;0]; % Start position agent 3
nav_p.xd4 = [env_size;-0.1;0]; % Goal position agebt 3

nav_p.p = 2; % Generate obstacle set off p-norm

% Target set radius | Radius when to stop using Density FB control
nav_p.rad_from_goal = 2; % Original value: 1.0


% General function: f(x) = ke^(-a/(1-b(x-c)^2)) + d
% Function: f(x):= k/exp(a/(1-||x-c||^2)) - k/e^a
nav_p.r11 = 0.5; nav_p.r12 = 2;
nav_p.r21 = 0.5; nav_p.r22 = 2;
nav_p.r31 = 0.5; nav_p.r32 = 2;
nav_p.r41 = 0.5; nav_p.r42 = 2;

% Obstancle centers (e.g. c1 and c2) | Domain R^n
c1 = nav_p.x01; 
c2 = nav_p.x02;
c3 = nav_p.x03; 
c4 = nav_p.x04;
nav_p.c0 = 0; % Add weaker condition -> unsafe set

theta = 0*pi/180; % [Radian] CounterClockwise | Rotate after
stretch = [1;1];
gamma = 0*pi/180; % [rad] CCW | Post rotate

% Function: g(x):= 1/||x||^alpha
nav_p.alpha = 0.4; % Best value 0.2

% Euler Parameters
M = 10000; %loop iterations
deltaT = 0.01;
ctrl_multiplier = 25; % Parameter to change

% Optimize function handle generation
vpa_enable = true;

if vpa_enable
    optimize = false; % Set optimize also false s.t. symbolic tool generation is fast / manageable
end

%% Density Function Formulation
% Create function of density and bump function
syms x [2,1] real
[A, A_inv] = transformationMatrix(theta, stretch, 2);

%% Form system matrices and LQR Gain
[single_int_p, dbl_int_p] = generateStateSpace(deltaT, x);

x1_temp = nav_p.x01;
x2_temp = nav_p.x02;
x3_temp = nav_p.x03;
x4_temp = nav_p.x04;

x_euler1 = zeros(M,size(nav_p.x01,1));
u_euler1 = zeros(M,2);

x_euler2 = zeros(M,size(nav_p.x02,1));
u_euler2 = zeros(M,2);

x_euler3 = zeros(M,size(nav_p.x03,1));
u_euler3 = zeros(M,2);

x_euler4 = zeros(M,size(nav_p.x04,1));
u_euler4 = zeros(M,2);

% create function handle for bump functions and gradients
bumpHandles = createBumpHandles();
gradDensityHandles = createGradientHandles(nav_p);

for iter = 1:M
    if mod(iter,1) == 0
        iter
    end

    % using Euler method
    
    [x_euler1(iter,:), u_euler1(iter,:)] = forwardEuler_multiagent(nav_p, deltaT, ctrl_multiplier, ...
                        @unicycle_multiagent,gradDensityHandles,c1,c2,c3,c4, single_int_p,x1_temp,1);
    x1_temp = x_euler1(iter,:)';
    
    [x_euler2(iter,:), u_euler2(iter,:)] = forwardEuler_multiagent(nav_p, deltaT, ctrl_multiplier, ...
                        @unicycle_multiagent,gradDensityHandles,c1,c2,c3,c4, single_int_p,x2_temp,2);
    x2_temp = x_euler2(iter,:)';
    
    [x_euler3(iter,:), u_euler3(iter,:)] = forwardEuler_multiagent(nav_p, deltaT, ctrl_multiplier, ...
                        @unicycle_multiagent,gradDensityHandles,c1,c2,c3,c4, single_int_p,x3_temp,3);
    x3_temp = x_euler3(iter,:)';
    
    [x_euler4(iter,:), u_euler4(iter,:)] = forwardEuler_multiagent(nav_p, deltaT, ctrl_multiplier, ...
                        @unicycle_multiagent,gradDensityHandles,c1,c2,c3,c4, single_int_p,x4_temp,4);
    x4_temp = x_euler4(iter,:)';
    
    c1 = x1_temp; 
    c2 = x2_temp;
    c3 = x3_temp; 
    c4 = x4_temp;

end

%% plot time domain
figure()
t = 1:size(x_euler1,1);
subplot(2,2,1)
plot(t,[x_euler1(:,1),x_euler1(:,2)],'LineWidth',2);
ylabel('states')
xlabel('timesteps')
box on
subplot(2,2,2)
plot(t,[x_euler2(:,1),x_euler2(:,2)],'LineWidth',2);
ylabel('states')
xlabel('timesteps')
box on
subplot(2,2,3)
plot(t,[x_euler3(:,1),x_euler3(:,2)],'LineWidth',2);
ylabel('states')
xlabel('timesteps')
box on
subplot(2,2,4)
plot(t,[x_euler4(:,1),x_euler4(:,2)],'LineWidth',2);
ylabel('states')
xlabel('timesteps')
box on

figure()
subplot(2,2,1)
plot(t,u_euler1(:,1),'LineWidth',2);
ylabel('velocity')
xlabel('timesteps')
box on
subplot(2,2,2)
plot(t,u_euler2(:,1),'LineWidth',2);
ylabel('velocity')
xlabel('timesteps')
box on
subplot(2,2,3)
plot(t,u_euler3(:,1),'LineWidth',2);
ylabel('velocity')
xlabel('timesteps')
box on
subplot(2,2,4)
plot(t,u_euler4(:,1),'LineWidth',2);
ylabel('velocity')
xlabel('timesteps')
box on

figure()
subplot(2,2,1)
plot(t,u_euler1(:,2),'LineWidth',2);
ylabel('omega')
xlabel('timesteps')
box on
subplot(2,2,2)
plot(t,u_euler2(:,2),'LineWidth',2);
ylabel('omega')
xlabel('timesteps')
box on
subplot(2,2,3)
plot(t,u_euler3(:,2),'LineWidth',2);
ylabel('omega')
xlabel('timesteps')
box on
subplot(2,2,4)
plot(t,u_euler4(:,2),'LineWidth',2);
ylabel('omega')
xlabel('timesteps')
box on


%% Animation
skip_rate = 10;
save_videos = true;

% Define the video file path and name
video_file_path = sprintf('animations/scenario.mp4'); % Adjust path if necessary

if save_videos
    vidFile = VideoWriter(video_file_path, 'MPEG-4');
    vidFile.FrameRate = 5;
    open(vidFile);
end

% jj := frame #
for jj = 1:skip_rate:M
    f1 = figure(999);
    clf(f1); % Clear current figure
    set(f1, 'color', 'white'); % Set figure to white

    hold on;

    % Plot each vehicle's state as a circle with heading line
    % Vehicle 1
    plotCircleAndHeading(x_euler1(jj,1), x_euler1(jj,2), x_euler1(jj,3), ...
        nav_p.r11, nav_p.r12, 'red');
    % Vehicle 2
    plotCircleAndHeading(x_euler2(jj,1), x_euler2(jj,2), x_euler2(jj,3), ...
        nav_p.r21, nav_p.r22, 'green');
    % Vehicle 3
    plotCircleAndHeading(x_euler3(jj,1), x_euler3(jj,2), x_euler3(jj,3), ...
        nav_p.r31, nav_p.r32, 'blue');
    % Vehicle 4
    plotCircleAndHeading(x_euler4(jj,1), x_euler4(jj,2), x_euler4(jj,3), ...
        nav_p.r41, nav_p.r42, 'magenta');

    % Plot the desired positions
    plot(nav_p.xd1(1), nav_p.xd1(2), 'or', 'MarkerSize', 10, 'MarkerFaceColor', 'red'); 
    plot(nav_p.xd2(1), nav_p.xd2(2), 'og', 'MarkerSize', 10, 'MarkerFaceColor', 'green'); 
    plot(nav_p.xd3(1), nav_p.xd3(2), 'ob', 'MarkerSize', 10, 'MarkerFaceColor', 'blue'); 
    plot(nav_p.xd4(1), nav_p.xd4(2), 'om', 'MarkerSize', 10, 'MarkerFaceColor', 'magenta'); 
    
    % Set plot properties
    title('Multiagent obstacle avoidance');
    box on
    set(gca, 'xtick', []);
    set(gca, 'ytick', []);
    axis square;
    axis tight;
    xlim([-10, 10]);
    ylim([-10, 10]);

    % Add timestamp text
    timestamp = sprintf('Time: %0.2f s', deltaT * jj);
    text(0.6, 0.1, timestamp, 'Units', 'normalized');

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

%% Function to plot circle, heading line, and sensing region
function plotCircleAndHeading(x, y, theta, r1, r2, color)
    % Plot the sensing region as a filled circle with transparency
    theta_fill = linspace(0, 2*pi, 100);
    x_fill = r2 * cos(theta_fill) + x;
    y_fill = r2 * sin(theta_fill) + y;
    fill(x_fill, y_fill, color, 'FaceAlpha', 0.3, 'EdgeColor', 'none'); % Adjust 'FaceAlpha' for transparency

    % Plot the robot's circle
    rectangle('Position', [x-r1, y-r1, 2*r1, 2*r1], ...
              'Curvature', [1, 1], 'EdgeColor', color, 'LineWidth', 1.5);

    % Compute the end point of the heading line
    line_end_x = x + 1.5 * r1 * cos(theta); % Adjust line length if needed
    line_end_y = y + 1.5 * r1 * sin(theta);

    % Plot the heading line
    plot([x, line_end_x], [y, line_end_y], 'Color', color, 'LineWidth', 1.5);
end

%%
function bumpHandles = createBumpHandles()
    % Define the function handles for each bump
    syms x [2,1] real
    bump1 = @(nav_p,c1,c2,c3,c4) formPNormBump(nav_p.r21, nav_p.r22, c2, x, nav_p.p, true) ...
                                  .* formPNormBump(nav_p.r31, nav_p.r32, c3, x, nav_p.p, true) ...
                                  .* formPNormBump(nav_p.r41, nav_p.r42, c4, x, nav_p.p, true);
    
    bump2 = @(nav_p,c1,c2,c3,c4) formPNormBump(nav_p.r11, nav_p.r12, c1, x, nav_p.p, true) ...
                                  .* formPNormBump(nav_p.r31, nav_p.r32, c3, x, nav_p.p, true) ...
                                  .* formPNormBump(nav_p.r41, nav_p.r42, c4, x, nav_p.p, true);
    
    bump3 = @(nav_p,c1,c2,c3,c4) formPNormBump(nav_p.r11, nav_p.r12, c1, x, nav_p.p, true) ...
                                  .* formPNormBump(nav_p.r21, nav_p.r22, c2, x, nav_p.p, true) ...
                                  .* formPNormBump(nav_p.r41, nav_p.r42, c4, x, nav_p.p, true);
    
    bump4 = @(nav_p,c1,c2,c3,c4) formPNormBump(nav_p.r11, nav_p.r12, c1, x, nav_p.p, true) ...
                                  .* formPNormBump(nav_p.r21, nav_p.r22, c2, x, nav_p.p, true) ...
                                  .* formPNormBump(nav_p.r31, nav_p.r32, c3, x, nav_p.p, true);

    bumpHandles.bump1Handle = bump1;
    bumpHandles.bump2Handle = bump2;
    bumpHandles.bump3Handle = bump3;
    bumpHandles.bump4Handle = bump4;
end

function gradDensityHandles = createGradientHandles(nav_p)
    bumpHandles = createBumpHandles();
    syms x [2,1] real
    syms c1 [2,1] real
    syms c2 [2,1] real
    syms c3 [2,1] real
    syms c4 [2,1] real

    % form V(x) for each agent
    g1 = 1/norm(x(1:2)-nav_p.xd1(1:2))^(2*nav_p.alpha); 
    g2 = 1/norm(x(1:2)-nav_p.xd2(1:2))^(2*nav_p.alpha); 
    g3 = 1/norm(x(1:2)-nav_p.xd3(1:2))^(2*nav_p.alpha);
    g4 = 1/norm(x(1:2)-nav_p.xd4(1:2))^(2*nav_p.alpha); 

    % Compute densities using bump handles
    grad_density1 = gradient(g1 * bumpHandles.bump1Handle(nav_p,c1,c2,c3,c4),x);
    grad_density2 = gradient(g2 * bumpHandles.bump2Handle(nav_p,c1,c2,c3,c4),x);
    grad_density3 = gradient(g3 * bumpHandles.bump3Handle(nav_p,c1,c2,c3,c4),x);
    grad_density4 = gradient(g4 * bumpHandles.bump4Handle(nav_p,c1,c2,c3,c4),x);

    % Create function handles
    optimize = false;
    grad_density1_sym = matlabFunction(grad_density1, 'File', 'functions/grad_density_f1', 'Vars', {x,c1,c2,c3,c4}, 'Optimize', optimize);
    grad_density2_sym = matlabFunction(grad_density2, 'File', 'functions/grad_density_f2', 'Vars', {x,c1,c2,c3,c4}, 'Optimize', optimize);
    grad_density3_sym = matlabFunction(grad_density3, 'File', 'functions/grad_density_f3', 'Vars', {x,c1,c2,c3,c4}, 'Optimize', optimize);
    grad_density4_sym = matlabFunction(grad_density4, 'File', 'functions/grad_density_f4', 'Vars', {x,c1,c2,c3,c4}, 'Optimize', optimize);

    % Compute gradients of densities with respect to x
    gradDensityHandles.grad_density1 = @(x,c1,c2,c3,c4) grad_density1_sym(x,c1,c2,c3,c4);
    gradDensityHandles.grad_density2 = @(x,c1,c2,c3,c4) grad_density2_sym(x,c1,c2,c3,c4);
    gradDensityHandles.grad_density3 = @(x,c1,c2,c3,c4) grad_density3_sym(x,c1,c2,c3,c4);
    gradDensityHandles.grad_density4 = @(x,c1,c2,c3,c4) grad_density4_sym(x,c1,c2,c3,c4);
end