clear all
close all

function [foot_positions, is_right_foot] = generate_foot_trajectory_2d(dx, dy, p_curr, is_right_foot, step_length, step_width)
    
    % Per salvare le posizioni e orientamenti [x, y, theta, is_right]
    foot_positions = []; 
    eps = 1E-4;
    
    % Calcola il vettore tangente alla traiettoria
    tangent = [dx, dy];
    tangent = tangent / (norm(tangent) + eps); % Normalizza
    
    % Calcola il vettore normale (perpendicolare alla tangente)
    normal = [-tangent(2), tangent(1)]; % Rotazione di 90 gradi
    
    % Determina la posizione del piede
    x_foot = p_curr(1) + (step_width) * normal(1) * (2 * ~is_right_foot - 1);
    y_foot = p_curr(2) + (step_width) * normal(2) * (2 * ~is_right_foot - 1);
    
    % Calcola l'orientamento del piede (angolo rispetto all'asse x globale)
    theta_foot = atan2(tangent(2), tangent(1));
    
    % Salva posizione e orientamento del piede
    foot_positions = [x_foot, y_foot, theta_foot, is_right_foot];
    
    % Alterna tra piede destro e sinistro
    is_right_foot = ~is_right_foot;

end


function [foot_positions, is_right_foot] = generate_foot_trajectory_rotation(p_curr, is_right_foot, step_width)
      
    % Calcola l'orientamento del robot all'asse x globale
    theta_robot = p_curr(4); 
    
    % Calcola il vettore direzione corrispondente
    tangent = [cos(theta_robot), sin(theta_robot)];
    
    % Calcola il vettore normale (perpendicolare a orientamento robot)
    normal = [-tangent(2), tangent(1)]; % Rotazione di 90 gradi
    
    % Calcola l'orientamento del piede (angolo rispetto all'asse x globale)
    theta_foot = theta_robot;  
    
    % angle perpendicular to robot orientation
    theta_norm = atan2(normal(2), normal(1));
    
    % Determina la posizione del piede
    x_foot = p_curr(1) + (step_width) * cos(is_right_foot*pi + theta_norm);
    y_foot = p_curr(2) + (step_width) * sin(is_right_foot*pi + theta_norm);
    
    % Salva posizione e orientamento del piede
    foot_positions = [x_foot, y_foot, theta_foot, is_right_foot];
    
    % Alterna tra piede destro e sinistro
    is_right_foot = ~is_right_foot;
       
end


function [translated_corners] = visualize_foot_rectangle(x_foot, y_foot, theta_foot, foot_size)

    % Calcola i vertici del rettangolo del piede
    half_length = foot_size(1) / 2;
    half_width = foot_size(2) / 2;
    % Vertici rispetto al centro del piede
    corners = [-half_length, -half_width;
                half_length, -half_width;
                half_length,  half_width;
               -half_length,  half_width]; 

    % Applica rotazione e traslazione ai vertici
    rotation_matrix = [cos(theta_foot), -sin(theta_foot);
                       sin(theta_foot),  cos(theta_foot)];
    rotated_corners = (rotation_matrix * corners')';
    translated_corners = rotated_corners + [x_foot, y_foot];
          
end



% sampling time
delta_T = 0.01; % seconds

% Parameters for footsteps
step_length = 0.4; % Distance along the walking direction
step_width = 0.2; % Lateral offset from CoM
foot_size = [0.15, 0.05]; % [Length, Width] of foot rectangles

% theta step length when rotating on the spot
step_length_theta = 0.4; % [rad]

% Alternate left/right foot
is_right_foot = true;
dist_accumulated = 0; % Accumulatore per la distanza
dist_accumulated_theta = 0;

% Input commands (simulating user input)
##v1 = [1.0, 0.0, 0.5].*ones(3.0/delta_T, 1);
##v2 = [1.0, 1.0, 0.0].*ones(3.0/delta_T, 1);
##v3 = [0.0, 0.0, 0.5].*ones(3.0/delta_T, 1);
##v4 = [1.0, 0.0, 0.5].*ones(4.0/delta_T, 1);
##v = [v1; v2; v3; v4];
v1 = [1.0, 0.0, 0.5].*ones(3.0/delta_T, 1);
v2 = [1.0, 1.0, 0.0].*ones(3.0/delta_T, 1);
v3 = [0.0, 0.0, 0.5].*ones(5.0/delta_T, 1);
v4 = [1.0, 0.0, 0.5].*ones(4.0/delta_T, 1);
v = [v1; v2; v3; v4];

% Number of points
num_points = length(v);

% simulation time
T = num_points*delta_T;

% Initialize CoM position (start from zero)
X_g = 0.0; 
Y_g = 0.0;
Z_g = 1.0; % Not used in 2D plot
theta = 0;

% Store CoM trajectory
trajectory = zeros(num_points, 4);

% status: 0: normal, 1: zigzag, 2: rotation on spot
status = -1;
status_prec = -1;

% Initialize plotting
figure;
hold on;
plot_handle = plot(0, 0, 'r-', 'LineWidth', 2); % CoM trajectory
title_handle = title('', 'FontSize', 18); % Placeholder for dynamic title
xlabel('X Position (m)');
ylabel('Y Position (m)');
grid on;
axis equal;
xlim([-4, 4]);
ylim([-1, 7]);

% Calculate CoM trajectory and plot footsteps
for i = 2:num_points
    % Extract velocities
    Vx = v(i, 1);
    Vy = v(i, 2);
    Wz = v(i, 3);
    
    % Transform to global frame
    Vx_g = Vx * cos(theta) - Vy * sin(theta);
    Vy_g = Vx * sin(theta) + Vy * cos(theta);
    
    % Update global position and orientation
    X_g = X_g + Vx_g * delta_T;
    Y_g = Y_g + Vy_g * delta_T;
    theta = theta + Wz * delta_T;
    
    % Store trajectory
    trajectory(i, :) = [X_g, Y_g, Z_g, theta];
    
    % Plot CoM trajectory
    set(plot_handle, 'XData', trajectory(1:i, 1), 'YData', trajectory(1:i, 2));
    set(title_handle, 'String', sprintf('V_x: %.2f m/s, V_y: %.2f m/s, W_z: %.2f rad/s', Vx, Vy, Wz));

    % Posiziona un nuovo passo se la distanza accumulata supera step_length
    if (Vx ~= 0 || Vy ~= 0) 
        
##        status = 0;
##                
##        if (status ~= status_prec)
##           % generate footstep 
##          [foot_positions, is_right_foot] = generate_foot_trajectory_rotation(trajectory(i,:), is_right_foot, step_width);
##          
##          % get foot corners for visualization
##          [translated_corners] = visualize_foot_rectangle(foot_positions(1), foot_positions(2), foot_positions(3), foot_size);
##          
##          % plot 2d footsteps
##          scatter(foot_positions(:, 1), foot_positions(:, 2), 5, 'filled', 'MarkerEdgeColor', 'k'); % Posizioni dei piedi
##          
##          % Disegna il rettangolo
##          fill(translated_corners(:, 1), translated_corners(:, 2), 'b', 'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineWidth', 1.5);
##        endif
##        
##        status_prec = status;
        
        % Calcolo della distanza tra punti consecutivi
        dx = trajectory(i, 1) - trajectory(i-1, 1);
        dy = trajectory(i, 2) - trajectory(i-1, 2);
        dist_step = sqrt(dx^2 + dy^2);
    
        dist_accumulated = dist_accumulated + dist_step;
        
        if (dist_accumulated >= step_length)
        
          % Resetta la distanza accumulata
          dist_accumulated = 0; 
          
          % generate footstep 
          [foot_positions, is_right_foot] = generate_foot_trajectory_2d(dx, dy, [X_g, Y_g], is_right_foot, step_length, step_width);
          
          % get foot corners for visualization
          [translated_corners] = visualize_foot_rectangle(foot_positions(1), foot_positions(2), foot_positions(3), foot_size);
          
          % plot 2d footsteps
          scatter(foot_positions(:, 1), foot_positions(:, 2), 5, 'filled', 'MarkerEdgeColor', 'k'); % Posizioni dei piedi
          
          % Disegna il rettangolo
          fill(translated_corners(:, 1), translated_corners(:, 2), 'b', 'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineWidth', 1.5);   
        
        endif
      
    elseif (Vx == 0 && Vy == 0 && Wz ~= 0)
      
##        status = 2;
##        
##        if (status ~= status_prec)
##           % generate footstep 
##          [foot_positions, is_right_foot] = generate_foot_trajectory_rotation(trajectory(i,:), is_right_foot, step_width, foot_size);
##          
##          % get foot corners for visualization
##          [translated_corners] = visualize_foot_rectangle(foot_positions(1), foot_positions(2), foot_positions(3), foot_size);
##          
##          % plot 2d footsteps
##          scatter(foot_positions(:, 1), foot_positions(:, 2), 5, 'filled', 'MarkerEdgeColor', 'k'); % Posizioni dei piedi
##          
##          % Disegna il rettangolo
##          fill(translated_corners(:, 1), translated_corners(:, 2), 'b', 'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineWidth', 1.5);
##        endif
##        
##        status_prec = status;
        
        % Calcolo della distanza tra punti consecutivi  
        dist_accumulated_theta = dist_accumulated_theta + Wz*delta_T;
        
        if (abs(dist_accumulated_theta) >= step_length_theta)
          
          % Resetta la distanza accumulata
          dist_accumulated_theta = 0;
          
          % generate footstep 
          [foot_positions, is_right_foot] = generate_foot_trajectory_rotation(trajectory(i,:), is_right_foot, step_width, foot_size);
          
          % get foot corners for visualization
          [translated_corners] = visualize_foot_rectangle(foot_positions(1), foot_positions(2), foot_positions(3), foot_size);
          
          % plot 2d footsteps
          scatter(foot_positions(:, 1), foot_positions(:, 2), 5, 'filled', 'MarkerEdgeColor', 'k'); % Posizioni dei piedi
          
          % Disegna il rettangolo
          fill(translated_corners(:, 1), translated_corners(:, 2), 'b', 'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineWidth', 1.5);
          
        endif
        
    end
    
    % Pause for real-time animation
    pause(0.02);
end

hold off;