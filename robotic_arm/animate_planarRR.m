function animate_planarRR(t,joints,navigation_params,obs_funs,ee_plan,x_ini )

%% get robot params
[robot_params,euler_params,~,~,animate_params] = get_params();

%% setup gif
f1 = figure(999);
set(f1, 'color', 'white') % Set figure to white
% f1.WindowState = 'maximized';
flag_movie = animate_params.flag_movie;
skip_rate = animate_params.skip_rate; %skip this many frames per loop
if flag_movie
    try
        name = ['animations/planarRR.mp4'];
        vidfile = VideoWriter(name,'MPEG-4');
    catch ME
        name = ['/animations/planarRR'];
        vidfile = VideoWriter(name,'Motion JPEG AVI');
    end
    open(vidfile);
end

%% get parameeters
a1 = robot_params.l1 ; a2 = robot_params.l2;
dT = euler_params.step_size;
n_obs = navigation_params.n_obs;
x_obs = navigation_params.x_obs;
x_obs_rad = navigation_params.x_obs_rad;

N = length(t)-1;
grayColor = [.6 .6 .6];

%% plot first frame

q1 = joints(1,1); q2 = joints(1,2);
x1 = a1*sin(q1); y1 = -a1*cos(q1);
x2 = x1 + a2*sin(q1+q2); y2 = y1 - a2*cos(q1+q2);

% plot starting position of end effector
plot(x_ini(1),x_ini(2),'or', 'MarkerSize',10, 'MarkerFaceColor','red'); hold on

% plot obstacles at t=0;
if(navigation_params.dynamic_obs)
    for n = 1:params.n_obs
        % get cornors of a rectanlge
        obs_t = obs_funs{n}(0)';
        r = x_obs_rad(n); d = 2*r; 
        px = obs_t(1)-r; py = obs_t(2)-r; 
        obs(n) = rectangle('Position',[px py d d],'Curvature',[1,1],'FaceColor',grayColor, 'LineWidth',2);
    end
    
else
    for n = 1:n_obs
        % get cornors of a rectanlge
        r = x_obs_rad(n); d = 2*r; 
        px = x_obs(n,1)-r; py = x_obs(n,2)-r; 
        obs = rectangle('Position',[px py d d],'Curvature',[1,1],'FaceColor',grayColor,'LineWidth',2);
    end
end

% plot initial link position
link1 = line([0 x1],[0 y1],'color','k','LineWidth',10);
link2 = line([x1 x2],[y1 y2],'color','r','LineWidth',10);

% record time
T = t;
T_end = t(end);
str = strcat("Time: " + num2str(T) + " s" + "/", num2str(T_end+" s"));
timestamp = text(1.5,-2.5,str,'HorizontalAlignment','left','VerticalAlignment','top');
axis([-3 3 -3 3])
axis square

%% loop through frames
for i = 1:skip_rate:N
    % plot goal position on the end effector
    t_goal = i*dT;
    x_ref = x_ref_f(t_goal);
    plot(x_ref(1), x_ref(2),'og', 'MarkerSize',10, 'MarkerFaceColor','green'); 

    %  plot trail for ee traj
    if(~isempty(ee_plan))
        plot(ee_plan.x(1:i,1),ee_plan.x(1:i,2), 'black', 'LineWidth', 2, 'LineStyle','--');
    end

    % update obstacle positions
    if(navigation_params.dynamic_obs)
        for n = 1:params.n_obs
            obs_t = obs_funs{n}(i*dT)';
            r = x_obs_rad(n); d = 2*r; 
            px = obs_t(1)-r; py = obs_t(2)-r;  
            set(obs(n),'Position',[px py d d],'Curvature',[1 1],'FaceColor',grayColor);
        end
    end

    q1 = joints(i,1); q2 = joints(i,2);
    x1 = a1*sin(q1); y1 = -a1*cos(q1);
    x2 = x1 + a2*sin(q1+q2); y2 = y1 - a2*cos(q1+q2);
    
    % update link positions
    set(link1,'xdata',[0 x1],'ydata',[0 y1]);
    set(link2,'xdata',[x1 x2],'ydata',[y1 y2]);
    
    % update time
    T = t(i);
    T_end = t(end);
    str = strcat("Time: " + num2str(T) + " s" + "/", num2str(T_end+" s"));
    set(timestamp,'String',str);
    
    pause(0.05);
    
    if flag_movie
            writeVideo(vidfile, getframe(gcf));
    end
    drawnow
end

if flag_movie
    close(vidfile);
end

end