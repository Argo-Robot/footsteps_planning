%% Generate footsteps, CoM and ZMP trajectories for a biped given a global path.

clear all
close all


function [trajectory] = generate_global_trajectory(v,w,T, delta_T)
  
    % Generate a global trajectory given v,w as input.
      
    % Number of points
    num_points = T/delta_T;
    
    % Init starting position
    pos_x = 0.0; 
    pos_y = 0.0;
    pos_z = 1.0;
    theta = 0.0;
    
    % Store trajectory values
    trajectory = zeros(num_points, 4);
    
    % Compute trajectory
    for i = 1:num_points      
        pos_x = pos_x + v*cos(theta)*delta_T;
        pos_y = pos_y + v*sin(theta)*delta_T;
        theta = theta + w*delta_T;
        
        trajectory(i,:) = [pos_x, pos_y, pos_z, theta];
    end
    
    % Visualization
    figure;
    hold on;
    plot(trajectory(:, 1), trajectory(:, 2), 'r-', 'LineWidth', 2); % CoM
    title('Global trajectory', 'FontSize', 15 );
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    grid on;
    axis equal;
end


function [CoM_trajectory] = generate_pelvis_trajectory(trajectory, T, delta_T)
    
    % Generate a CoM trajectory for pelvis, by applying a sine wave perturbation
    % around the original global trajectory.
    
    % Parameters
    A = 0.2; % Amplitude of sine wave
    lambda = 1.0; % Wavelength of sine wave
    
    % Number of points
    num_points = T/delta_T;
    
    x_base = trajectory(:,1); % x(t)
    y_base = trajectory(:,2); % y(t)
    
    % Calculate arc length
    s = zeros(1, num_points);
    
    % Get arc length values
    for i = 2:num_points
        s(i) = s(i-1) + sqrt((x_base(i) - x_base(i-1))^2 + (y_base(i) - y_base(i-1))^2);
    end
    
    % Uniformly sample arc length
    s_uniform = linspace(s(1), s(end), num_points);
    x_uniform = interp1(s, x_base, s_uniform);
    y_uniform = interp1(s, y_base, s_uniform);
    
    % Compute tangent and normal vectors
    dx_ds = gradient(x_uniform, s_uniform);
    dy_ds = gradient(y_uniform, s_uniform);
    tangent_magnitude = sqrt(dx_ds.^2 + dy_ds.^2);
    t_x = dx_ds ./ tangent_magnitude;
    t_y = dy_ds ./ tangent_magnitude;
    n_x = -t_y;
    n_y = t_x;
    
    % Apply sine wave perturbation along normal
    sine_wave = A * sin(2 * pi * s_uniform / lambda);
    x_perturbed = x_uniform + sine_wave .* n_x;
    y_perturbed = y_uniform + sine_wave .* n_y;
    z_perturbed = trajectory(:,3);
    
    % Generate homogeneous transformation matrices
    CoM_trajectory = cell(num_points, 1);
    
    for i = 1:num_points
        % Rotation matrix around z-axis
        R_z = [cos(trajectory(i,4)), -sin(trajectory(i,4)), 0;
               sin(trajectory(i,4)),  cos(trajectory(i,4)), 0;
                                  0,                     0, 1];
        
        % Homogeneous transformation matrix
        T = eye(4);
        T(1:3, 1:3) = R_z;
        T(1:3, 4) = [x_perturbed(i); y_perturbed(i); z_perturbed(i)];
        
        CoM_trajectory{i} = T;
    end
    
    % Plot results
    figure;
    hold on;
    plot(trajectory(:, 1), trajectory(:, 2), 'r-', 'LineWidth', 2); % CoM
    plot(x_perturbed, y_perturbed, 'b-', 'LineWidth', 2); % Perturbed trajectory
    title('CoM Trajectory', 'FontSize', 15 );
    xlabel('x');
    ylabel('y');
    legend('Original Trajectory', 'Perturbed Trajectory');
    grid on;
    axis equal;
end


function foot_positions = generate_foot_trajectory_2d(trajectory, step_length, step_width, foot_size)
    
    % Generate 2D foot positions around the global trajectory.

    % Variables
    num_points = size(trajectory, 1);
    foot_positions = []; % [x, y, theta, is_right]
    dist_accumulated = 0; % Store accumulated distance
    is_right_foot = true; % Alternate between left and right foot

    for i = 2:num_points-1
        % Compute distance between consecutive points
        dx = trajectory(i+1, 1) - trajectory(i, 1);
        dy = trajectory(i+1, 2) - trajectory(i, 2);
        dist_step = sqrt(dx^2 + dy^2);
        
        % Update accumulated distance
        dist_accumulated = dist_accumulated + dist_step;
        
        % Check if it is time to place a new step
        if dist_accumulated >= step_length
            
            % Reset variable
            dist_accumulated = 0; 
            
            % Compute vector tangent to trajectory
            tangent = [dx, dy];
            tangent = tangent / norm(tangent); % Normalizza
            
            % Compute vector normal to trajectory
            normal = [-tangent(2), tangent(1)]; 
            
            % Compute foot position
            x_foot = trajectory(i, 1) + (step_width / 2) * normal(1) * (2 * is_right_foot - 1);
            y_foot = trajectory(i, 2) + (step_width / 2) * normal(2) * (2 * is_right_foot - 1);
            
            % Compute orientation of the foot
            theta_foot = atan2(tangent(2), tangent(1));
            
            % Save data
            foot_positions = [foot_positions; x_foot, y_foot, theta_foot, is_right_foot];
            
            % Alternate steps
            is_right_foot = ~is_right_foot;
        end
    end

    % Visualization
    figure;
    hold on;
    
    for i = 1:size(foot_positions, 1)
        
        % Plot rectangles of the feet
        x_foot = foot_positions(i, 1);
        y_foot = foot_positions(i, 2);
        theta_foot = foot_positions(i, 3);
        
        % Compute rectangle vertices
        half_length = foot_size(2) / 2;
        half_width = foot_size(1) / 2;
        corners = [
            -half_length, -half_width;
             half_length, -half_width;
             half_length,  half_width;
            -half_length,  half_width
        ]; 

        % Do rotation and translation
        rotation_matrix = [cos(theta_foot), -sin(theta_foot);
                           sin(theta_foot),  cos(theta_foot)];
        rotated_corners = (rotation_matrix * corners')';
        translated_corners = rotated_corners + [x_foot, y_foot];
        
        % Draw rectangle
        fill(translated_corners(:, 1), translated_corners(:, 2), 'b', 'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineWidth', 1.5);
    end
    
    % Plot results
    plot(trajectory(:, 1), trajectory(:, 2), 'r--', 'LineWidth', 1.5); 
    scatter(foot_positions(:, 1), foot_positions(:, 2), 5, 'filled', 'MarkerEdgeColor', 'k'); 
    axis equal;
    title('Foot Placement 2D', 'FontSize', 15 );
    xlabel('x-axis');
    ylabel('y-axis');
    grid on;
end


function foot_trajectory = generate_foot_trajectory_3d(foot_positions, step_height, step_length, step_time, delta_T)
    
    % Generate 3D foot positions starting from 2D positions.
    
    % Number of points
    num_points = step_time/delta_T;
    
    t = linspace(0, 1, num_points); % Normalized time
    foot_trajectory = cell(size(foot_positions, 1) - 2, 1); % Trajectory for each step

    for i = 1:size(foot_positions, 1) - 2
        % Initial and final step positions
        p_start = foot_positions(i, 1:2); 
        p_end = foot_positions(i + 2, 1:2); 
        
        % Interpolate with cubix spline for xy direction
        x_traj = interp1([0, 1], [p_start(1), p_end(1)], t, 'pchip');
        y_traj = interp1([0, 1], [p_start(2), p_end(2)], t, 'pchip');
        
        % Parabolic trajectory for z direction
        z_traj = 4 * step_height * t .* (1 - t); % Profilo parabolico
        
        % Compute transformation matrix
        HTM_trajectory = cell(num_points, 1);
        yaw_angle = foot_positions(i + 1);
        
        for j = 1:num_points
            % Rotation matrix around z
            R_z = [cos(yaw_angle), -sin(yaw_angle), 0;
                   sin(yaw_angle),  cos(yaw_angle), 0;
                   0,                0,               1];
            
            % Transformation matrix
            T = eye(4);
            T(1:3, 1:3) = R_z;
            T(1:3, 4) = [x_traj(j); y_traj(j); z_traj(j)];
            HTM_trajectory{j} = T;
        end
        
        % Save trajectory
        foot_trajectory{i} = HTM_trajectory;
    end
    
    % Visualization
    figure;
    hold on;
    for i = 1:length(foot_trajectory)
        traj = foot_trajectory{i};
        x = arrayfun(@(j) traj{j}(1, 4), 1:num_points);
        y = arrayfun(@(j) traj{j}(2, 4), 1:num_points);
        z = arrayfun(@(j) traj{j}(3, 4), 1:num_points);
        plot3(x,y,z, 'LineWidth', 1.5);
    end
    axis equal;
    title('Foot Trajectory 3D');
    xlabel('x-axis');
    ylabel('y-axis');
    zlabel('z-axis');
    grid on;
end


function zmp_trajectory = generate_zmp_trajectory_spline(foot_positions, delta_size_spline, step_length, step_width, foot_size)
  
    % Generate ZMP trajectory given the 2D footsteps.
    
    % Get feet positions.
    x_foot = foot_positions(:, 1);
    y_foot = foot_positions(:, 2);

    % Add origin (0, 0) as first point in the trajectory.
    x_foot = [0; x_foot];
    y_foot = [0; y_foot];
    
    % Temporal parameter 
    t = 1:length(x_foot); 
    t_fine = 1:delta_size_spline:length(x_foot); % Greater granularity.
    
    % Cubic interpolation between feet positions.
    x_spline = interp1(t, x_foot, t_fine); 
    y_spline = interp1(t, y_foot, t_fine); 

    % Store results in matrix
    zmp_trajectory = [x_spline', y_spline'];

    % Visualize ZMP trajectory
    figure;
    hold on;
    plot(zmp_trajectory(:, 1), zmp_trajectory(:, 2), 'k-', 'LineWidth', 1.5); % ZMP spline
    
    for i = 1:size(foot_positions, 1)
        % Plot rectangles of the feet
        x_foot = foot_positions(i, 1);
        y_foot = foot_positions(i, 2);
        theta_foot = foot_positions(i, 3);
        
        % Compute rectangle vertices
        half_length = foot_size(2) / 2;
        half_width = foot_size(1) / 2;
        corners = [
            -half_length, -half_width;
             half_length, -half_width;
             half_length,  half_width;
            -half_length,  half_width
        ]; 

        % Do rotation and translation
        rotation_matrix = [cos(theta_foot), -sin(theta_foot);
                           sin(theta_foot),  cos(theta_foot)];
        rotated_corners = (rotation_matrix * corners')';
        translated_corners = rotated_corners + [x_foot, y_foot];
        
        % Draw rectangle
        fill(translated_corners(:, 1), translated_corners(:, 2), 'b', 'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineWidth', 1.5);
    end
    
    axis equal;
    title('ZMP trajectory', 'FontSize', 15 );
    xlabel('x-axis');
    ylabel('y-axis');
    grid on;
end



function CoM_trajectory = generate_CoM_trajectory_from_ZMP(zmp_trajectory, h, delta_t)
  
    # Alternative approach: generate CoM trajectory for pelvis by exploiting dynamics 
    % equations related to ZMP.
    
    % Gravity constant
    g = 9.81; 
    
    % Number of points
    num_points = size(zmp_trajectory, 1);
    
    % Init CoM position and velocity
    vel_x = 0;
    vel_y = 0;
    pos_x = zmp_trajectory(1, 1); % Initial CoM position is the initial ZMP position
    pos_y = zmp_trajectory(1, 2); % Same for Y
    
    % Store values
    CoM_trajectory = zeros(num_points, 2);
    
    % Initial CoM position is the initial ZMP position
    CoM_trajectory(1, :) = [pos_x, pos_y];
    
    % Compute CoM trajectory
    for i = 2:num_points
        
        % CoM acceleration 
        acc_x = (g/h)*(zmp_trajectory(i, 1) - pos_x);
        acc_y = (g/h)*(zmp_trajectory(i, 2) - pos_y);
    
        % Compute CoM velocity by integrating CoM acceleration 
        vel_x = vel_x + acc_x * delta_t;
        vel_y = vel_y + acc_y * delta_t;
        
        % Compute CoM position by integrating CoM velocity 
        pos_x = pos_x + vel_x * delta_t;
        pos_y = pos_y + vel_y * delta_t;
        
        % Update CoM trajectory
        CoM_trajectory(i, :) = [pos_x, pos_y];
    end
    
    % Visualization of CoM and ZMP
    figure;
    plot(zmp_trajectory(:, 1), zmp_trajectory(:, 2), 'r-', 'LineWidth', 2); % ZMP
    hold on;
    plot(CoM_trajectory(:, 1), CoM_trajectory(:, 2), 'b-', 'LineWidth', 2); % CoM
    title('ZMP and CoM Trajectory', 'FontSize', 15 );
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    legend('ZMP Trajectory', 'CoM Trajectory');
    grid on;
    axis equal;
end




%% MAIN

% trajectory variables
v = 1.5; % [m/s]
w = 0.7; % [rad/s]
T = 5.0; % [s]
delta_T = 0.01; % [s]

% gait variables
step_length = 0.4; % [m]
step_width = 0.2; % [m]
foot_size = [0.1, 0.2]; % width x length

% generate global trajectory based on v,w
trajectory = generate_global_trajectory(v, w, T, delta_T);

% generate CoM  in world frame
CoM_trajectory = generate_pelvis_trajectory(trajectory, T, delta_T);

% generate 2D foot positions in world frame
foot_positions = generate_foot_trajectory_2d(trajectory, step_length, step_width, foot_size);

% generate 3D foot trajectories in world frame
step_height = 0.1; % [m]
step_time = 1.0; % [s]
foot_trajectory = generate_foot_trajectory_3d(foot_positions, step_height, step_length, step_time, delta_T);

% generate ZMP trajectory
delta_size_spline = 0.01;
zmp_trajectory = generate_zmp_trajectory_spline(foot_positions, delta_size_spline, step_length, step_width, foot_size);

% alternative: generate CoM trajectory by using dynamics relations with ZMP
h_com = 3.0; % [m]
CoM_trajectory_dynamics = generate_CoM_trajectory_from_ZMP(zmp_trajectory, h_com, delta_T);








