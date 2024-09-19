%% Obstacle Avoidance for planar robotic arm
clc; clear; close all
addpath(genpath('./navigation'))
addpath(genpath('robotic_arm'))
addpath(genpath('saved_data'))

% Delete the funcs before running
delete('navigation\func\*')
% else run twice for changes in density to update

% use motion plan or not
use_motion_plan = false;

%% working obstacles and configurations
% 1 random
%navigation_params.x_obs = [-0.7, 0.5];
%navigation_params.x_ini = IK_planarRR([-0.5 -0.6]);
%navigation_params.x_goal = IK_planarRR([-0.5 0.8]);

% 2 random
%navigation_params.x_obs = [1.2, -0.5];
%navigation_params.x_ini = IK_planarRR([-1 -1]);
%navigation_params.x_goal =[0.95 2.4];

% 3 swing up**
%navigation_params.x_obs = [1.2, -0.5];
%navigation_params.x_ini = IK_planarRR([0 -2]);
%navigation_params.x_goal =IK_planarRR([0 2]);

% 4 start close to obstacle
%navigation_params.x_obs = [1.2, -0.5; ...
%                          -1.2, -0.5];
% navigation_params.x_ini = IK_planarRR([0.5 -0.1]);
% navigation_params.x_goal =IK_planarRR([0 2]);

%% Problem setup
x = sym('x',[2 1],'real'); %joint states
[robot_params,euler_params,navigation_params,lqr_params,animate_params] = get_params(x);
animate_params.skip_rate=1;
syms t;
% define start and goal in joint space
navigation_params.x_ini = [0.01 0.01];
t_scale = 1;
time_varying_goal = [0.8+sin(t_scale*t), -0.6-cos(t_scale*t)]; % good one - circle
navigation_params.x_goal = IK_planarRR(time_varying_goal);

% goal velocity
navigation_params.x_vel_goal_f =  diff(navigation_params.x_goal);

% goal accel
navigation_params.x_accel_goal_f =  diff(navigation_params.x_vel_goal_f);


% define obstacles
navigation_params.dynamic_obs = 0;
navigation_params.n_obs = 2;
navigation_params.x_obs = [1.5, -0.5; ...
                          -1.5, -0.5];
navigation_params.x_obs_rad = 0.2*ones(navigation_params.n_obs);

params.flag_movie = 1;
params.dyanmic_traj = 1;
params.density_gif = 0;
params.static_obs = 0;

%% get obs in joint space (uncomment to generate obs for new configuration)
% [joint_obs, convex_hull] = gen_joint_obs(navigation_params);
% save('saved_data/joint_obs','joint_obs')

%% get density function
disp('--- getting density function ---')
load('saved_data/joint_obs','joint_obs')
navigation_funs = get_density_joint2(navigation_params,joint_obs,use_motion_plan);

%% Create function handles
obs_funs = [];
matlabFunction(navigation_params.x_goal, 'File', 'navigation/func/xd_f', 'Vars', {t}, 'Optimize', false);
matlabFunction(navigation_params.x_vel_goal_f, 'File', 'navigation/func/x_dot_d_f', 'Vars', {t}, 'Optimize', false);
matlabFunction(navigation_params.x_accel_goal_f, 'File', 'navigation/func/x_ddot_d_f', 'Vars', {t}, 'Optimize', false);
matlabFunction(jacobian(navigation_params.x_goal, t), 'File', 'navigation/func/jac_xd_f', 'Vars', {t}, 'Optimize', false);
matlabFunction(navigation_funs.piecewise_f, 'File', './navigation/func/bump_f', 'Vars', {x,t}, 'Optimize', false);
matlabFunction(navigation_funs.V, 'File', './navigation/func/rho_f', 'Vars', {x,t}, 'Optimize', false);
matlabFunction(navigation_funs.density, 'File', './navigation/func/density_f', 'Vars', {x,t}, 'Optimize', false);
matlabFunction(navigation_funs.grad_density, 'File', './navigation/func/grad_density_f', 'Vars', {x,t}, 'Optimize', false);
rehash path

%% using Euler method to get plan
if(use_motion_plan)
    disp('--- getting motion plan ---')
    [x_euler, u_euler] = forwardEuler(@singleIntegrator,navigation_params);
    x_euler = x_euler';
end

%% get inverse dynamics 
if(use_motion_plan)
    % inverse dynamics with motion planner
    disp('--- running inverse dynamics controller with motion plan ---')
    q_des = x_euler; q_dot_des = u_euler; q_ddot_des = [diff(u_euler(1,:));diff(u_euler(2,:))]; 
else
    % inverse dynamics without motion plan
    disp('--- running inverse dynamics controller without motion plan ---')
    q_des = navigation_params.x_goal'; q_dot_des = [0;0]; q_ddot_des = [0;0]; 
end    
joint_control = ID_planarRR(q_des,q_dot_des,q_ddot_des,robot_params,navigation_params,euler_params,use_motion_plan);

%% plot density
% plot_density_joint(x_euler, navigation_params, obs_funs)
% plot_density_joint(joint_control.angles, navigation_params, obs_funs)

%% animate control
ee_plan = [];
x_ini = FK_planarRR(navigation_params.x_ini - [pi/2 0]);
x_goal = FK_planarRR(navigation_params.x_goal - [pi/2 0]);
navigation_params.time_varying_goal = time_varying_goal;

joints = joint_control.angles;
joint_control.t = euler_params.step_size:euler_params.n_steps;
time = joint_control.t;
animate_planarRR(time,joints,navigation_params,obs_funs,ee_plan,x_ini,t_scale,x_goal)

%% save plan and paraters
% save('saved_data/navigation_params','navigation_params')

%% plots for paper
plotfigure2(robot_params,navigation_params,joint_control,joints);