function joint_ID = ID_planarRR(q_des,q_dot_des,q_ddot_des,robot_params,navigation_params,euler_params,use_motion_plan)

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

        % Density based inverse dynamics control
        if(use_motion_plan)
            % default controller
            % e = q - q_des(i,:)'; e_dot = q_dot - q_dot_des(:,i);
            % grad_density = grad_density_f(e,t);
            % Kv = 10;
            % u_id = M*(q_ddot_des(:,i)) + C + G + M*(grad_density(1:2) -Kv.*e_dot);
            
            % vanilla inverse dynamics
            Kp  = 1; Kv = 10;
            e = q - q_des(i,:)'; e_dot = q_dot - q_dot_des(:,i);
            u_id = M*(q_ddot_des(:,i)) + C + G + M*(-Kp*e -Kv.*e_dot);

            % % Density controller with backstepping
            % e = q - q_des(i,:)'; e_dot = q_dot - q_dot_des(:,i);
            % grad_density = grad_density_f(q,t);
            % density = density_f(q,t);
            % alpha_bar = 2;
            % norm_e = norm(e)^2;
            % scale = 0.5;
            % % scale = 0.1;
            % beta2=1;
            % u_id = M*(beta2*q_ddot_des(:,i)) + C + G + M*(scale*norm_e*grad_density(1:2)./density);

        else
            % new density controller without motion plan
            Kp = 1; Kv = 1/25; beta2 = 1; beta3 = 10;
            e = q - navigation_params.q_goal(i,:)'; 
            e_dot = q_dot - q_dot_des(i,:)';
            grad_density = grad_density_f(q,t);
            u_bar = beta2*(Kp.*grad_density(1:2) - Kv.*e_dot);
            u_id = beta3*M*q_ddot_des(i,:)' + C + G + M*u_bar;
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
