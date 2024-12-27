%% file

clear all
close all






function foot_positions = plan_foot_trajectory_2d(trajectory, step_length, step_width, delta_dist, foot_size)
    
    % TODO: I passi generati non mantengono step_length, da fizare.
    % Comunque questo è per caso semplice lineare, con traiettorie curve necessito di 
    % gestione più eleborata con matrici rotazione e tangenti
    
    % Calcolo delle posizioni dei piedi lungo una traiettoria 2D
    % trajectory: Nx2 matrice dei punti della traiettoria globale [x, y]
    % step_length: distanza tra passi consecutivi lungo x
    % step_width: distanza laterale tra i piedi
    % delta_dist: passo della traiettoria originale (risoluzione)
    % foot_size: dimensione del piede [lunghezza, larghezza]
    
    num_points = size(trajectory, 1);
    foot_positions = []; % Per salvare le posizioni [x, y, is_right]
    dist_accumulated = 0; % Accumulatore di distanza
    is_right_foot = true; % Alterna tra piede destro e sinistro

    for i = 1:num_points-1
        % Calcolo della distanza tra i punti successivi
        dx = trajectory(i+1, 1) - trajectory(i, 1);
        dy = trajectory(i+1, 2) - trajectory(i, 2);
        dist_step = sqrt(dx^2 + dy^2);
        
        dist_accumulated = dist_accumulated + dist_step;
        
        % Controlla se è necessario posizionare un nuovo piede
        if i==1 || dist_accumulated >= step_length
            % Azzera il contatore della distanza accumulata
            dist_accumulated = 0;
            
            % Determina la posizione del piede
            x_foot = trajectory(i, 1);
            if is_right_foot
                y_foot = trajectory(i, 2) + step_width / 2;
            else
                y_foot = trajectory(i, 2) - step_width / 2;
            end
            
            % Aggiungi la posizione del piede alla lista
            foot_positions = [foot_positions; x_foot, y_foot, is_right_foot];
            
            % Alterna tra piede destro e sinistro
            is_right_foot = ~is_right_foot;
        end
    end

    % Visualizzazione
    figure;
    hold on;
    for i = 1:size(foot_positions, 1)
        rectangle('Position', [foot_positions(i, 1) - foot_size(2)/2, ...
                               foot_positions(i, 2) - foot_size(1)/2, ...
                               foot_size(2), foot_size(1)], ...
                  'EdgeColor', 'b', 'LineWidth', 1.5);
    end
    plot(trajectory(:, 1), trajectory(:, 2), 'r--', 'LineWidth', 1.5); % Traccia originale
    axis equal;
    title('Foot Placement (2D)');
    xlabel('x-axis');
    ylabel('y-axis');
    grid on;
end



function foot_trajectory = generate_foot_trajectory_3d(foot_positions, step_height, step_length, num_points)
    foot_trajectory = cell(size(foot_positions, 1), 1); % Store trajectories
    t = linspace(0, 1, num_points); % Normalized time

    for i = 1:size(foot_positions, 1)
        x_traj = linspace(foot_positions(i, 1), foot_positions(i, 1) + step_length, num_points);
        y_traj = linspace(foot_positions(i, 2), foot_positions(i, 2), num_points);
        z_traj = 4 * step_height * t .* (1 - t); % Parabolic height profile
        foot_trajectory{i} = [x_traj', y_traj', z_traj'];
    end

    % Visualization
    figure;
    hold on;
    for i = 1:length(foot_trajectory)
        traj = foot_trajectory{i};
        plot3(traj(:, 1), traj(:, 2), traj(:, 3), 'LineWidth', 1.5);
    end
    axis equal;
    title('Foot Trajectory (3D)');
    xlabel('x-axis');
    ylabel('y-axis');
    zlabel('z-axis');
    grid on;
end


function zmp_trajectory = generate_zmp_trajectory_spline(foot_positions, delta_size_spline)
    % Genera una traiettoria ZMP utilizzando una spline cubica
    % foot_positions: Nx2 matrice delle posizioni dei piedi [x, y]
    % delta_size_spline: granularità della traiettoria ZMP
    
    % Estraggo le coordinate X e Y dai piedi
    x_foot = foot_positions(:, 1);
    y_foot = foot_positions(:, 2);

    % Aggiungo il punto di origine (0, 0) come primo punto della traiettoria
    x_foot = [0; x_foot];
    y_foot = [0; y_foot];
    
    % Creazione del parametro temporale t per i nodi dei piedi
    t = 1:length(x_foot); % Parametro temporale per i nodi dei piedi
    t_fine = 1:delta_size_spline:length(x_foot); % Maggiore granularità
    
    % Interpolazione spline cubica per X e Y
    x_spline = interp1(t, x_foot, t_fine); % Spline cubica per X
    y_spline = interp1(t, y_foot, t_fine); % Spline cubica per Y

    % Combino i risultati in una matrice ZMP
    zmp_trajectory = [x_spline', y_spline'];

    % Visualizzazione della traiettoria ZMP
    figure;
    hold on;
    plot(zmp_trajectory(:, 1), zmp_trajectory(:, 2), 'k-', 'LineWidth', 1.5); % ZMP spline
    
    % Disegna i piedi come rettangoli
    for i = 1:size(foot_positions, 1)
        rectangle('Position', [foot_positions(i, 1) - 0.1, ...
                               foot_positions(i, 2) - 0.05, ...
                               0.2, 0.1], ... % 20cm x 10cm come esempio
                  'EdgeColor', 'b', 'LineWidth', 1.5);
    end
    axis equal;
    title('ZMP Trajectory Using Cubic Spline');
    xlabel('x-axis');
    ylabel('y-axis');
    grid on;
end



function CoM_trajectory = generate_CoM_trajectory_from_ZMP(zmp_trajectory, h, delta_t)
    % h: altezza del CoM (in metri)
    % delta_t: passo temporale (in secondi)
    % zmp_trajectory: traiettoria ZMP (Nx2: [x, y])
    
    % Costante gravitazionale
    g = 9.81; 
    
    % Numero di punti
    num_points = size(zmp_trajectory, 1);
    
    % Inizializzazione velocità e posizione CoM (partiamo da zero)
    vel_x = 0;
    vel_y = 0;
    pos_x = zmp_trajectory(1, 1); % La posizione iniziale del CoM è la ZMP iniziale
    pos_y = zmp_trajectory(1, 2); % Allo stesso modo per Y
    
    % Memorizzazione dei valori della traiettoria del CoM
    CoM_trajectory = zeros(num_points, 2);
    
    % Primo punto di CoM è preso come la posizione della ZMP
    CoM_trajectory(1, :) = [pos_x, pos_y];
    
    % Calcolo della traiettoria del CoM
    for i = 2:num_points
        
        % Accelerazione del CoM
        acc_x = (g/h)*(zmp_trajectory(i, 1) - pos_x);
        acc_y = (g/h)*(zmp_trajectory(i, 2) - pos_y);
    
        % Calcolo velocità tramite integrazione della accelerazione
        vel_x = vel_x + acc_x * delta_t;
        vel_y = vel_y + acc_y * delta_t;
        
        % Calcolo posizione tramite integrazione della velocità
        pos_x = pos_x + vel_x * delta_t;
        pos_y = pos_y + vel_y * delta_t;
        
        % Aggiornamento della traiettoria del CoM
        CoM_trajectory(i, :) = [pos_x, pos_y];
    end
    
    % Visualizzazione della traiettoria ZMP e CoM
    figure;
    plot(zmp_trajectory(:, 1), zmp_trajectory(:, 2), 'r--', 'LineWidth', 2); % ZMP
    hold on;
    plot(CoM_trajectory(:, 1), CoM_trajectory(:, 2), 'b-', 'LineWidth', 2); % CoM
    title('ZMP and CoM Trajectory');
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    legend('ZMP Trajectory', 'CoM Trajectory');
    grid on;
    axis equal;
end




% global trajectory
delta_dist = 0.1;
global_trajectory = [0:delta_dist:5; zeros(1, 51)]'; % lungo x

% gait variables
step_length = 0.4;
step_width = 0.2;
foot_size = [0.1, 0.2]; % 10cm x 20cm foot

% Plan 2D trajectory for feet
foot_positions = plan_foot_trajectory_2d(global_trajectory, step_length, step_width, delta_dist, foot_size);

% Generate 3D trajectory for feet
step_height = 0.1; % 10cm step height
num_points = 50;
foot_trajectory = generate_foot_trajectory_3d(foot_positions, step_height, step_length, num_points);

% calcolo traiettoria ZMP
% Parametro di granularità
delta_size_spline = 0.01;

% Genera la traiettoria ZMP
zmp_trajectory = generate_zmp_trajectory_spline(foot_positions, delta_size_spline);

% calcolo CoM trajectory
h_com = 1.5;
delta_t = 0.1;
CoM_trajectory = generate_CoM_trajectory_from_ZMP(zmp_trajectory, h_com, delta_t)




