function joint_ID = ID_planarRR(q_des,q_dot_des,q_ddot_des,robot_params,navigation_params,euler_params,use_motion_plan)

    Kp = 1; %gain for testing regular inverse dynamics
    Kv = 1; %gain (Kv=10 working for density based inverse dynamics)
    q = navigation_params.x_ini'; q_dot = [0;0];
    joint_ID.angles = q';
    joint_ID.vel = q_dot';
    joint_ID.control = [0;0]';
    dt = euler_params.step_size; N = euler_params.n_steps;
    
    % wrap around SE2 torous for q1 and q2
    if(q(1) > 2*pi)
        disp('wrap:0');
        q(1) = q(1,1) - 2*pi; 
    end
    if(q(2,1) > 2*pi)
        disp('wrap:0')
        q(2) = q(2,1) - 2*pi; 
    end
    if(q(1) < 0)
        disp('wrap:0')
        q(1) = 2*pi + q(1,1);
    end
    if(q(2) < 0)
        disp('wrap:0')
        q(2) = 2*pi + q(2,1);
    end

    t = 0;
    for i = 1:N-2
        x = [q;q_dot];  
        % get M C G matrices
        dynamics = dynamics_planarRR(0, x, [0;0], robot_params);
        M = dynamics.M; C = dynamics.C; G = dynamics.G;

        % vanilla inverse dynamics
        % e = q - q_des(i,:)'; e_dot = q_dot - q_dot_des(:,i);
        % u_id = M*(q_ddot_des(:,i)) + C + G + M*(-Kp*e -Kv.*e_dot);

        % Density based inverse dynamics control
        if(use_motion_plan)
            e = q - q_des(i,:)'; e_dot = q_dot - q_dot_des(:,i);
            grad_density = grad_density_f(e,t);
            u_id = M*(q_ddot_des(:,i)) + C + G + M*(grad_density(1:2) -Kv.*e_dot);
        else
            % new density controller without motion plan
            e = q - navigation_params.q_goal(i,:)'; 
            e_dot = q_dot - navigation_params.q_dot_goal(i,:)';
            q_ddot_des = navigation_params.q_ddot_goal(i,:)';
            grad_density = grad_density_f(q,t);
            u_bar = Kp.*grad_density(1:2) -Kv.*e_dot;
            u_id = navigation_params.ctrl_multiplier*(M*q_ddot_des + C + G + M*u_bar);
        

        % Density controller with backstepping
%         grad_density = grad_density_f(q,t);
%         density = density_f(q,t);
%         alpha_bar = 10;
%         z = q_dot - q_des(i,:);
%         norm_z = norm(z)^2;
%         scale = 1/(2*alpha_bar - 2);
%         u_id = M*(q_ddot_des(:,i)) + C + G + M*(scale*norm_z*grad_density(1:2)/density);

        end

        % apply control to simulate using euler
        dynamics = dynamics_planarRR(t, x, u_id, robot_params);
        x_update = x + dt.*dynamics.f;
        
        % update states
        q = x_update(1:2); q_dot = x_update(3:4);
        
         % wrap around SE2 torous for q1 and q2
        if(q(1) > 2*pi)
            disp('wrap:'); disp(i);
            q(1) = q(1,1) - 2*pi; 
        end
        if(q(2) > 2*pi)
            disp('wrap:'); disp(i);
            q(2) = q(2,1) - 2*pi;
        end
        if(q(1) < 0)
            disp('wrap:'); disp(i);
            q(1) = 2*pi + q(1,1);
        end
         if(q(2) < 0)
            disp('wrap:'); disp(i);
            q(2) = 2*pi + q(2,1);
        end 

        joint_ID.angles = [joint_ID.angles; q'];
        joint_ID.vel = [joint_ID.vel; q_dot'];
        joint_ID.control = [joint_ID.control; u_id'];
        t = t + dt;
    end
end
