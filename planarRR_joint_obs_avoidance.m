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

%% Problem setup
x = sym('x',[2 1],'real'); %joint states
[robot_params,euler_params,navigation_params,lqr_params,animate_params] = get_params(x);

syms t;
% define start and goal in joint space
navigation_params.x_ini = [0.01 0.01];
t_scale = 1;
% time_varying_goal = [0.8+sin(t_scale*t), -0.6-cos(t_scale*t)]; % good one - circle
% time_varying_goal = [0.5+sin(t_scale*t), -0.4-cos(t_scale*t)]; % good one - circle
% time_varying_goal = [0.6+sin(t_scale*t), -0.6-cos(t_scale*t)]; % good one - circle
time_varying_goal = [3*sin(exp(-5/(t_scale*t)))-1, -1]; % good one - horizontal line
% time_varying_goal = [1.3, 2*sin(exp(-5/(t_scale*t^2)))-1.5]; % good one - vertical line

% time_varying_goal = [sin(t_scale*t),sin(t_scale*t)-0.5]; % good one - diag line
% time_varying_goal = [sin(1.5*exp(-1/(t_scale*t)))-0.5,sin(1.5*exp(-1/(t_scale*t)))-1.7]; % good one - diag line
% time_varying_goal = [2*sin(exp(-1/(t_scale*t)))-0.5, sin(exp(-1/(t_scale*t)))-0.5]; 
matlabFunction(time_varying_goal, 'File', 'navigation/func/x_ref_f', 'Vars', {t}, 'Optimize', false);

% get ref traj in joint space as funtion
navigation_params.x_goal_f = IK_planarRR(time_varying_goal);
navigation_params.x_vel_goal_f =  diff(navigation_params.x_goal_f);
navigation_params.x_accel_goal_f =  diff(navigation_params.x_vel_goal_f);

% get values of vel and accel as vector
if(~use_motion_plan )
    disp('--- getting joint reference ---')
    t_vector = (0:euler_params.n_steps).*euler_params.step_size;
    q_goal = [];
    for i = 1:length(t_vector)
         q_goal = [q_goal;IK_planarRR(x_ref_f(t_vector(i)))];
    end
    navigation_params.q_goal = q_goal;
    navigation_params.q_dot_goal = diff(q_goal);
    navigation_params.q_ddot_goal = diff(navigation_params.q_dot_goal);

    figure()
    subplot(3,1,1)
    plot(t_vector, navigation_params.q_goal,'-o')
    subplot(3,1,2)
    plot(t_vector(1:end-1), navigation_params.q_dot_goal,'-o')
    subplot(3,1,3)
    plot(t_vector(1:end-2), navigation_params.q_ddot_goal,'-o')

end

% define obstacles
navigation_params.dynamic_obs = 0;
navigation_params.n_obs = 2;
navigation_params.x_obs = [1.5, -0.5; ...
                          -1.5, -0.5];
navigation_params.x_obs_rad = 0.2*ones(navigation_params.n_obs);

% other params
params.flag_movie = 1;
params.dyanmic_traj = 1;
params.density_gif = 0;
params.static_obs = 0;
animate_params.skip_rate=1;

%% get obs in joint space (uncomment to generate obs for new configuration)
% [joint_obs, convex_hull] = gen_joint_obs(navigation_params);
% save('saved_data/joint_obs','joint_obs')

%% get density function
disp('--- getting density function ---')
load('saved_data/joint_obs','joint_obs')
navigation_funs = get_density_joint2(navigation_params,joint_obs,use_motion_plan);

%% Create function handles
obs_funs = [];

% joint space reference traj functions
matlabFunction(navigation_params.x_goal_f, 'File', 'navigation/func/xd_f', 'Vars', {t}, 'Optimize', false);
matlabFunction(navigation_params.x_vel_goal_f, 'File', 'navigation/func/x_dot_d_f', 'Vars', {t}, 'Optimize', false);
matlabFunction(navigation_params.x_accel_goal_f, 'File', 'navigation/func/x_ddot_d_f', 'Vars', {t}, 'Optimize', false);
matlabFunction(jacobian(navigation_params.x_goal_f, t), 'File', 'navigation/func/jac_xd_f', 'Vars', {t}, 'Optimize', false);

% density based functions
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
    q_des = navigation_params.q_goal; 
    q_dot_des = navigation_params.q_dot_goal; 
    q_ddot_des = navigation_params.q_ddot_goal; 
end    
joint_control = ID_planarRR(q_des,q_dot_des,q_ddot_des,robot_params,navigation_params,euler_params,use_motion_plan);

%% plot density
% plot_density_joint(x_euler, navigation_params, obs_funs)
% plot_density_joint(joint_control.angles, navigation_params, obs_funs)

%% animate control
ee_plan = [];
x_ini = FK_planarRR(navigation_params.x_ini - [pi/2 0]);
navigation_params.time_varying_goal = time_varying_goal;

joints = joint_control.angles;
joint_control.t = (0:euler_params.n_steps).*euler_params.step_size;
time = joint_control.t;
animate_planarRR(time,joints,navigation_params,obs_funs,ee_plan,x_ini)

%% save plan and paraters
% save('saved_data/navigation_params','navigation_params')

%% plots for paper
% plotfigure2(robot_params,navigation_params,joint_control,joints);