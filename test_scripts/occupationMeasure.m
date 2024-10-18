%% Obstacle Avoidance using single integrator dynamics
clc
clear
close all
%% Problem setup
mkdir('functions');
mkdir('utils');
mkdir('dynamics-control');
addpath('./functions');
addpath('./utils');
addpath('dynamics-control');

tic

% Start position
nav_p.x0 = [-1;2]; % 
nav_p.xd = [4;-3]; % Final goal position

% nav_p.x0 = [-5;-3];
% nav_p.xd = [5;5];
 
% Target set radius | Radius when to stop using Density FB control
nav_p.rad_from_goal = 0.01;

% Euler Parameters
N = 10000; %timesteps
deltaT = 0.01;
ctrl_multiplier = 50; % Parameter to change
nav_p.alpha = 0.2; % Best value 0.2
c0 = 0;

colors = colororder;
blue = colors(1,:);
red = colors(2,:);
yellow = colors(3,:);
green = colors(5,:);
obsColor = [.7 .7 .7]; % Obstacle color -> Grey

num_grid_points = 100;

%% Density Function Formulation
% Create function of density and bump function
syms x [2 1] real
p = 2;

n_obs = 1;
r1 = 2; r2 = 3; c1 = [0;0];

m = (norm((x-c1),p)^p-r1^p)/(r2^p-r1^p);
f = piecewise(m<=0, 0, exp(-1/m));
f_shift = piecewise(1-m <= 0, 0, exp(-1/(1-m)));
% Normalize the symmetric step function
g = f/(f+f_shift);
g = vpa(g,8);

piecewise_f = g + c0;

% Function: g(x):= 1/||x||^alpha
V = norm(x-nav_p.xd)^2;
g = 1/(V^nav_p.alpha);


% Density function
density = g*piecewise_f;

% Create function handles
matlabFunction(piecewise_f, 'File', 'functions/bump_f', 'Vars', {x});
matlabFunction(V, 'File', 'functions/V_f', 'Vars', {x});
matlabFunction(g, 'File', 'functions/g_f', 'Vars', {x});
matlabFunction(density, 'File', 'functions/rho_f', 'Vars', {x});

% gradients
grad_psi = gradient(piecewise_f, x);
grad_V = gradient(V, x);
grad_density = gradient(density, x);
hess_density = hessian(density, x);

% Create function handles
matlabFunction(grad_psi, 'File', 'functions/grad_psi_f', 'Vars', {x});
matlabFunction(grad_V, 'File', 'functions/grad_V_f', 'Vars', {x});
matlabFunction(grad_density, 'File', 'functions/grad_density_f', 'Vars', {x});
matlabFunction(hess_density, 'File', 'functions/hess_density_f', 'Vars', {x});

% control terms
u_term1 = -(nav_p.alpha/(V^(nav_p.alpha+1))) * grad_V * piecewise_f; %negative term
u_term2 = (1/(V^nav_p.alpha)) * grad_psi; % positive term
matlabFunction(u_term1, 'File', 'functions/u_term1_f', 'Vars', {x});
matlabFunction(u_term2, 'File', 'functions/u_term2_f', 'Vars', {x});

toc

%% Plotting Density Function & Rantzer's Condition
x = -linspace(-5,5,num_grid_points);
y = x;

[X,Y] = meshgrid(x,y);
rho_val = zeros(size(x));
grad_rho_x1 = zeros(size(x));
grad_rho_x2 = zeros(size(x));

for i=1:length(x)
    for j = 1:length(y)
        psi(i,j) = bump_f([x(j);y(i)]);
        psi_grad = grad_psi_f([x(j);y(i)]);
        v(i,j) = V_f([x(j);y(i)]);
        v_grad = grad_V_f([x(j);y(i)]);

        rho(i,j) = rho_f([x(j);y(i)]);
        rho_grad = grad_density_f([x(j);y(i)]);
        rho_grad_x1(i,j) = rho_grad(1);
        rho_grad_x2(i,j) = rho_grad(2);
    end
end

%% Form system matrices and LQR Gain
[single_int_p, dbl_int_p] = generateStateSpace(deltaT);

%% Euler with differnt initial conditions
% initial conditions around a box
num_pts = 100;
lx = [-4;-2]; ly = [2;4];
x = rand(1,num_pts)*range(lx) + lx(1);
y = rand(1,num_pts)*range(ly) + ly(1);
ics = [x;y];

num_ics = size(ics, 2); % Total number of ICs
x_euler_list = zeros(N+1, length(nav_p.xd), num_ics);
u_euler_list = zeros(N, length(nav_p.xd), num_ics);

for i = 1:num_ics
    nav_p.x0 = ics(:,i);
    [x_euler, u_euler] = forwardEuler(nav_p, N, deltaT, ctrl_multiplier, ...
                    @singleIntegrator, single_int_p);
    x_euler_list(:,:,i) = x_euler;
    u_euler_list(:,:,i) = u_euler;
end

%% Plot
hold on
f1 = figure(1);
set(f1, 'color', 'white') % Set figure to white
f1.WindowState = 'maximized';
subplot(2,4,1)
%quiverInLogScale(X, Y, Z_grad_x1, Z_grad_x2); hold on;
contour(X,Y,rho,'LineWidth',2,'LevelStep',0.2); hold on;

%plot obs
gamma = (0:100-1)*(2*pi/100);
points = c1 + [r1*cos(gamma);r1*sin(gamma)];
P = polyshape(points(1,:), points(2,:));
plot(P, 'FaceColor', obsColor, 'LineWidth', 2, 'FaceAlpha', 1.0); hold on;


%plot ics from a box
for i=1:num_ics
    plot(x_euler_list(:,1, i),x_euler_list(:,2, i),'-','color',red, 'LineWidth', 2); hold on;
end

%plot goal
plot(nav_p.xd(1), nav_p.xd(2), 'o', 'MarkerSize',10, 'MarkerFaceColor',green,'MarkerEdgeColor',green); hold on;

%plot dummy marker for obs
dummy_marker = plot(NaN,NaN, 'o','MarkerSize', 10, 'MarkerEdgeColor',...
            'black', 'MarkerFaceColor',obsColor, 'LineWidth', 1.5); hold on;

% plot options
axes1 = gca;
box(axes1,'on');
axis(axes1,'square');
hold(axes1,'off');
% Set the remaining axes properties
set(axes1,'FontSize',15,'LineWidth',1.5);

ylim([-5,5]);
xlim([-5,5]);
axis 'square'

xlabel('$x_1$','interpreter','latex', 'FontSize', 20);
ylabel('$x_2$','interpreter','latex', 'FontSize', 20);

%% plotting Density measure for simple setup
subplot2 = subplot(2,4,[2,6]);
surf(X,Y,log(rho), 'FaceAlpha',1, 'EdgeColor', 'k'); hold on
%contour(X,Y,rho,'LineWidth',2,'LevelStep',0.2,'ZLocation','zmin'); hold on;
colormap jet
zlim([-1,3])
xlim([-5,5])
ylim([-5,5])
xlabel('$x_1$','interpreter','latex', 'FontSize', 20);
ylabel('$x_2$','interpreter','latex', 'FontSize', 20);
zlabel('$\rho(x)$','interpreter','latex', 'FontSize', 20);
axis square
axes2 = gca;
set(axes2,'FontSize',15);
%view(2);
box(subplot2,'on');
%grid(subplot2,'on');
hold(subplot2,'off');

% Set the remaining axes properties
set(subplot2,'FontSize',15,'LineWidth',1.5,'TickDir','none')

%% plotting occupation measure for simple setup
x0 = ics;

x = -linspace(-5,5,num_grid_points);
y = x;

no_rows = length(x);%size(img,1);
no_columns = length(y);%size(img,2);

row_start = -5;
row_end = 5;
column_start = -5;
column_end = 5;

row_edges = linspace(row_start, row_end, no_rows);
column_edges = linspace(column_start, column_end, no_columns);
occupancy = zeros(no_rows,no_columns);


for j = 1:num_pts
    x_true = x0(:,j);
    x_open = x0(:,j);
    u_value = [];    
    nav_p.x0 = x_true;

   sol_traj = forwardEuler(nav_p, N, deltaT, ctrl_multiplier, @singleIntegrator, single_int_p);

    column_index = discretize(sol_traj(:,1),row_edges);
    row_index = discretize(sol_traj(:,2),column_edges);
    
    for i = 1:size(sol_traj,1)
        occupancy(row_index(i),column_index(i)) = occupancy(row_index(i),column_index(i)) + 1;
        occ = log(occupancy);
        occ(isinf(occ))=0;
    end
end
subplot(2,4,5)
hold on;
s = surf(row_edges,column_edges, occ);
xlabel('$x_1$','interpreter','latex');
ylabel('$x_2$','interpreter','latex');
zlabel('$\v(x)$','interpreter','latex');
axis square

%% gif for occupation measure
x0 = ics;

x = -linspace(-5,5,num_grid_points);
y = x;

no_rows = length(x);%size(img,1);
no_columns = length(y);%size(img,2);

row_start = -5;
row_end = 5;
column_start = -5;
column_end = 5;

row_edges = linspace(row_start, row_end, no_rows);
column_edges = linspace(column_start, column_end, no_columns);
occupancy = zeros(no_rows,no_columns);

occ = 0;

for j = 1:num_pts
    x_true = x0(:,j);
    x_open = x0(:,j);
    u_value = [];    
    nav_p.x0 = x_true;

   sol_traj = forwardEuler(nav_p, N, deltaT, ctrl_multiplier, @singleIntegrator, single_int_p);

    column_index = discretize(sol_traj(:,1),row_edges);
    row_index = discretize(sol_traj(:,2),column_edges);
    
    for i = 1:size(sol_traj,1)
        occupancy(row_index(i),column_index(i)) = occupancy(row_index(i),column_index(i)) + 1;
        occ = occ + log(occupancy);
        occ(isinf(occ))=0;
    end
    f2 = figure(2);
    set(f2, 'color', 'white') % Set figure to white
    clf(f2)
    f2.WindowState = 'maximized';
    s = surf(row_edges,column_edges, occ); 
    view(2)
    xlabel('$x_1$','interpreter','latex');
    ylabel('$x_2$','interpreter','latex');
    %zlabel('$\v(x)$','interpreter','latex');
axis square
end
