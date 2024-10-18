%% Obstacle Avoidance using single integrator dynamics
clc; clear; close all

% add paths
mkdir('animations');
addpath('./functions');
addpath('./utils');
addpath('dynamics-control');
addpath('./bump_lib');

%% Problem setup
% Navigation Parameters
% nav_p.x01 = [-3;0]; % Start position agent 1
% nav_p.xd1 = [3;0]; % Goal position agebt 1
% nav_p.x04 = [-0.5;-3]; % Start position agent 3
% nav_p.xd4 = [0.5;-3]; % Goal position agebt 3

nav_p.x01 = [0;2]; % Start position agent 1
nav_p.xd1 = [-0.01;-2]; % Goal position agebt 1
nav_p.x02 = [0;-2]; % Start position agent 2
nav_p.xd2 = [0.01;2]; % Goal position agebt 2
nav_p.x03 = [2;0]; % Start position agent 3
nav_p.xd3 = [-2;0.01]; % Goal position agebt 3
nav_p.x04 = [-2;0]; % Start position agent 3
nav_p.xd4 = [2;-0.01]; % Goal position agebt 3

nav_p.p = 2; % Generate obstacle set off p-norm

% Target set radius | Radius when to stop using Density FB control
nav_p.rad_from_goal = 0.1; % Original value: 1.0

% Ego Vehicle Size
nav_p.ego_r = 0.0; % Diameter of ego
sample_rate = 50; % Every sample_rate to plot trajectory of vehicle

% General function: f(x) = ke^(-a/(1-b(x-c)^2)) + d
% Function: f(x):= k/exp(a/(1-||x-c||^2)) - k/e^a
nav_p.r11 = 0.5; nav_p.r12 = 1;
nav_p.r21 = 0.5; nav_p.r22 = 1;
nav_p.r31 = 0.5; nav_p.r32 = 1;
nav_p.r41 = 0.5; nav_p.r42 = 1;

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
M = 1000; %loop iterations
deltaT = 0.01;
ctrl_multiplier = 80; % Parameter to change

% Optimize function handle generation
vpa_enable = true;

if vpa_enable
    optimize = false; % Set optimize also false s.t. symbolic tool generation is fast / manageable
end


%% Density Function Formulation
% Create function of density and bump function
syms x [2,1] real

[A, A_inv] = transformationMatrix(theta, stretch, 2);

g1 = 1/norm(x-nav_p.xd1)^(2*nav_p.alpha); % rho for agent 1
g2 = 1/norm(x-nav_p.xd2)^(2*nav_p.alpha); % rho for agent 2
g3 = 1/norm(x-nav_p.xd3)^(2*nav_p.alpha); % rho for agent 3
g4 = 1/norm(x-nav_p.xd4)^(2*nav_p.alpha); % rho for agent 4

%% Form system matrices and LQR Gain
[single_int_p, dbl_int_p] = generateStateSpace(deltaT, x);

x1_temp = nav_p.x01;
x2_temp = nav_p.x02;
x3_temp = nav_p.x03;
x4_temp = nav_p.x04;

x_euler1 = zeros(M,size(nav_p.x01,1));
u_euler1 = zeros(size(x_euler1));

x_euler2 = zeros(M,size(nav_p.x02,1));
u_euler2 = zeros(size(x_euler2));

x_euler3 = zeros(M,size(nav_p.x03,1));
u_euler3 = zeros(size(x_euler3));

x_euler4 = zeros(M,size(nav_p.x04,1));
u_euler4 = zeros(size(x_euler4));

% create function handle for bump functions and gradients
bumpHandles = createBumpHandles();
gradDensityHandles = createGradientHandles(nav_p);

for iter = 1:M
    if mod(iter,1) == 0
        iter
    end

    % using Euler method
    
    [x_euler1(iter,:), u_euler1(iter,:)] = forwardEuler_multiagent(nav_p, deltaT, ctrl_multiplier, ...
                        @singleIntegrator_multiagent,gradDensityHandles,c1,c2,c3,c4, single_int_p,x1_temp,1);
    x1_temp = x_euler1(iter,:)';
    
    [x_euler2(iter,:), u_euler2(iter,:)] = forwardEuler_multiagent(nav_p, deltaT, ctrl_multiplier, ...
                        @singleIntegrator_multiagent,gradDensityHandles,c1,c2,c3,c4, single_int_p,x2_temp,2);
    x2_temp = x_euler2(iter,:)';
    
    [x_euler3(iter,:), u_euler3(iter,:)] = forwardEuler_multiagent(nav_p, deltaT, ctrl_multiplier, ...
                        @singleIntegrator_multiagent,gradDensityHandles,c1,c2,c3,c4, single_int_p,x3_temp,3);
    x3_temp = x_euler3(iter,:)';
    
    [x_euler4(iter,:), u_euler4(iter,:)] = forwardEuler_multiagent(nav_p, deltaT, ctrl_multiplier, ...
                        @singleIntegrator_multiagent,gradDensityHandles,c1,c2,c3,c4, single_int_p,x4_temp,4);
    x4_temp = x_euler4(iter,:)';
    
    c1 = x1_temp; 
    c2 = x2_temp;
    c3 = x3_temp; 
    c4 = x4_temp;

end

%% Plot
colors = colororder;
blue = colors(1,:);
red = colors(2,:);
yellow = colors(3,:);
purple = colors(4,:);
green = colors(5,:);


% Parameter
state_index = 350+1; % use 50,150,210,350 for nice plots
trail_length = 150+1; %size(x_euler(1:state_index,:),1); % No trail -> trail_length = 1
tail_grad = linspace(0,1, trail_length);
gamma = (0:100-1)*(2*pi/100); % Angles for graphing circle

trail_skip_rate = 7;

% if trail_length == 1
%     trail_skip_rate = 1;
% else
%     trail_skip_rate = round(length(tail_grad)/10);
% end

figure1 = figure(1);
subplot(2,4,6)

% agent 1
% plot(x_euler1(1,1),x_euler1(1,2), 'square', 'MarkerSize', 10, 'MarkerEdgeColor', blue, 'MarkerFaceColor',blue); hold on;
% plot(nav_p.xd1(1),nav_p.xd1(2),'o', 'MarkerSize', 10, 'MarkerEdgeColor', 'black', 'MarkerFaceColor',blue); hold on;
s_plot = scatter(x_euler1(state_index-trail_length+1:trail_skip_rate:state_index,1),...
    x_euler1(state_index-trail_length+1:trail_skip_rate:state_index,2), 80,'filled','o','MarkerFaceColor',blue, 'LineWidth', 2); hold on;
s_plot.AlphaData = tail_grad(1:trail_skip_rate:trail_length);
s_plot.MarkerFaceAlpha = 'flat';
plot(x_euler1(state_index,1), x_euler1(state_index,2), 'o', 'MarkerSize', 10, 'MarkerEdgeColor', 'k', 'MarkerFaceColor',blue); hold on;
hold on;

% agent 2
% plot(x_euler2(1,1),x_euler2(1,2), 'square', 'MarkerSize', 10, 'MarkerEdgeColor', red, 'MarkerFaceColor',red'); hold on
% plot(nav_p.xd2(1),nav_p.xd2(2),'o', 'MarkerSize', 10, 'MarkerEdgeColor', 'black', 'MarkerFaceColor',red); hold on;
s_plot = scatter(x_euler2(state_index-trail_length+1:trail_skip_rate:state_index,1),...
    x_euler2(state_index-trail_length+1:trail_skip_rate:state_index,2), 80,'filled','o','MarkerFaceColor',red, 'LineWidth', 2); hold on;
s_plot.AlphaData = tail_grad(1:trail_skip_rate:trail_length);
s_plot.MarkerFaceAlpha = 'flat';
plot(x_euler2(state_index,1), x_euler2(state_index,2), 'o', 'MarkerSize', 10, 'MarkerEdgeColor', 'k', 'MarkerFaceColor',red); hold on;
hold on;

% agent 3
% plot(x_euler3(1,1),x_euler3(1,2), 'square', 'MarkerSize', 10, 'MarkerEdgeColor', yellow, 'MarkerFaceColor',yellow); hold on;
% plot(nav_p.xd3(1),nav_p.xd3(2),'o', 'MarkerSize', 10, 'MarkerEdgeColor', 'black', 'MarkerFaceColor',yellow); hold on;
s_plot = scatter(x_euler3(state_index-trail_length+1:trail_skip_rate:state_index,1),...
    x_euler3(state_index-trail_length+1:trail_skip_rate:state_index,2), 80,'filled','o','MarkerFaceColor',yellow, 'LineWidth', 2); hold on;
s_plot.AlphaData = tail_grad(1:trail_skip_rate:trail_length);
s_plot.MarkerFaceAlpha = 'flat';
plot(x_euler3(state_index,1), x_euler3(state_index,2), 'o', 'MarkerSize', 10, 'MarkerEdgeColor', 'k', 'MarkerFaceColor',yellow); hold on;
hold on;

% agent 4
% plot(x_euler4(1,1),x_euler4(1,2), 'square', 'MarkerSize', 10, 'MarkerEdgeColor', purple, 'MarkerFaceColor',purple); hold on;
% plot(nav_p.xd4(1),nav_p.xd4(2),'o', 'MarkerSize', 10, 'MarkerEdgeColor', 'black', 'MarkerFaceColor',purple); hold on;
s_plot = scatter(x_euler4(state_index-trail_length+1:trail_skip_rate:state_index,1),...
    x_euler4(state_index-trail_length+1:trail_skip_rate:state_index,2), 80,'filled','o','MarkerFaceColor',purple, 'LineWidth', 2); hold on;
s_plot.AlphaData = tail_grad(1:trail_skip_rate:trail_length);
s_plot.MarkerFaceAlpha = 'flat';
plot(x_euler4(state_index,1), x_euler4(state_index,2), 'o', 'MarkerSize', 10, 'MarkerEdgeColor', 'k', 'MarkerFaceColor',purple); hold on;
hold on;

%setup plots
axes1 = gca;
box(axes1,'on');
axis(axes1,'square');
hold(axes1,'off');
% Set the remaining axes properties
set(axes1,'FontSize',15,'LineWidth',1.5);
lgd.Interpreter = 'latex';
lgd.FontSize = 15;

%titleName = sprintf('Dyn Obstacles w/ dt = %0.1es, beta 2 = %0.2f ctrl mult = %0.1f', deltaT, beta_2, ctrl_multiplier);
% title(titleName);
xlabel('$x_1$','interpreter','latex', 'FontSize', 20);
ylabel('$x_2$','interpreter','latex', 'FontSize', 20);
xlim([-2,2]);
ylim([-2,2]);
axis square
hold on

% dummy plot for legend
subplot(2,4,3)
plot(x_euler1(1,1),x_euler1(1,2), 'square', 'MarkerSize', 10, 'MarkerEdgeColor', blue, 'MarkerFaceColor',blue); hold on;
plot(x_euler2(1,1),x_euler2(1,2), 'square', 'MarkerSize', 10, 'MarkerEdgeColor', red, 'MarkerFaceColor',red'); hold on;
plot(x_euler3(1,1),x_euler3(1,2), 'square', 'MarkerSize', 10, 'MarkerEdgeColor', yellow, 'MarkerFaceColor',yellow); hold on;
plot(x_euler4(1,1),x_euler4(1,2), 'square', 'MarkerSize', 10, 'MarkerEdgeColor', purple, 'MarkerFaceColor',purple); hold on;

%% Animation
skip_rate = 10;
save_videos = true;

multiagent_obstacle_avoidance = sprintf(...
    'animations/scenario3.mp4',...
    deltaT, ctrl_multiplier);

if save_videos
    vidFile = VideoWriter(multiagent_obstacle_avoidance, 'MPEG-4');
    vidFile.FrameRate = 5;
    open(vidFile);
end

% jj := frame #
for jj=1:skip_rate:M
    f1 = figure(999);
    clf(f1); % clear current figure
    set(f1, 'color', 'white') % Set figure to white

    plot(x_euler1(1,1),x_euler1(1,2), 'or', 'MarkerSize',10, 'MarkerFaceColor','red'); hold on;
    plot(nav_p.xd1(1),nav_p.xd1(2),'or', 'MarkerSize',10, 'MarkerFaceColor','red'); hold on;
    plot(x_euler2(1,1),x_euler2(1,2), 'og', 'MarkerSize',10, 'MarkerFaceColor','green'); hold on;
    plot(nav_p.xd2(1),nav_p.xd2(2),'og', 'MarkerSize',10, 'MarkerFaceColor','green'); hold on;
    plot(x_euler3(1,1),x_euler3(1,2), 'ob', 'MarkerSize',10, 'MarkerFaceColor','blue'); hold on;
    plot(nav_p.xd3(1),nav_p.xd3(2),'ob', 'MarkerSize',10, 'MarkerFaceColor','blue'); hold on;
    plot(x_euler4(1,1),x_euler4(1,2), 'om', 'MarkerSize',10, 'MarkerFaceColor','magenta'); hold on;
    plot(nav_p.xd4(1),nav_p.xd4(2),'om', 'MarkerSize',10, 'MarkerFaceColor','magenta'); hold on;
    
    plot(x_euler1(jj,1),x_euler1(jj,2), 'ok', 'MarkerSize',10, 'MarkerFaceColor','black');
    plot(x_euler2(jj,1),x_euler2(jj,2), 'ok', 'MarkerSize',10, 'MarkerFaceColor','black');
    plot(x_euler3(jj,1),x_euler3(jj,2), 'ok', 'MarkerSize',10, 'MarkerFaceColor','black');
    plot(x_euler4(jj,1),x_euler4(jj,2), 'ok', 'MarkerSize',10, 'MarkerFaceColor','black');
    
    titleName = sprintf('Multiagent obstacle avoidance');
    % title(titleName);
    % xlabel('$x_1$', 'interpreter', 'latex', 'FontSize', 20)
    % ylabel('$x_2$', 'interpreter', 'latex', 'FontSize', 20)
    set(gca, 'xtick', []);
    set(gca, 'ytick', []);
    axis square
    axis tight
    xlim([-5,5]);
    ylim([-5,5]);
    timestamp = sprintf("Time: %0.2f s", deltaT*jj);
    text(0.6,0.1, timestamp, 'Units', 'normalized');
    %obs_period = sprintf("Obstacle frequency: %0.2f Hz", nav_p.obstacle_freq);
    %text(0.6, 0.05, obs_period, 'Units', 'normalized');
    if save_videos
        writeVideo(vidFile, getframe(gcf));
    end
end

if save_videos
    close(vidFile);
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
    g1 = 1/norm(x-nav_p.xd1)^(2*nav_p.alpha); 
    g2 = 1/norm(x-nav_p.xd2)^(2*nav_p.alpha); 
    g3 = 1/norm(x-nav_p.xd3)^(2*nav_p.alpha);
    g4 = 1/norm(x-nav_p.xd4)^(2*nav_p.alpha); 

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