%% Generate an elliptical foot trajectory in 2D (xz plane) for a quadruped doing Trot Gait.

%% The front left leg and the rear right leg move together.
%% The front right leg and the rear left leg move together.

% Parameters for the elliptical foot trajectory
A_x = 0.1; % Amplitude in the forward (x) direction [meters]
A_z = 0.05; % Amplitude in the vertical (z) direction [meters]

% Define phase offsets for trot gait
phi_rear_rigth_0 = 0;         % Rear right leg initial phase
phi_front_right_0 = pi;       % Front right leg initial phase (opposite phase)

% Define stepping frequency
T0 = 3.0; # Gait cycle [s]
f0 = 1/T0; # Frequency [Hz] 

% Distance between diagonal legs in the x plane
offset_x = 0.2; % [meters]
offset_y = 0.0; % [meters]

num_steps = 100; % Number of steps in the trajectory
time = linspace(0, T0, num_steps);

% Function to calculate phase at time t
phase = @(phi_0, t) mod(phi_0 + 2 * pi * f0 * t, 2 * pi);

% Function to generate elliptical trajectory for a given phase
foot_trajectory = @(phi) [A_x * cos(phi); -A_z * sin(phi)]; 

% Initialize storage for trajectories
rear_rigth_traj = zeros(2, num_steps);
front_right_traj = zeros(2, num_steps);

% Compute trajectories for each leg
for i = 1:num_steps
    % Compute phase for each leg
    phi_rr = phase(phi_rear_rigth_0, time(i));
    phi_fr = phase(phi_front_right_0, time(i));
    
    % Compute foot trajectories
    rear_rigth_traj(:, i) = foot_trajectory(phi_rr) + [-offset_x; offset_y];
    front_right_traj(:, i) = foot_trajectory(phi_fr) + [offset_x; offset_y];
end

% Animation setup
figure;
hold on;
grid on;
axis equal;
xlim([-0.4, 0.4]);
ylim([-0.3, 0.3]);
xlabel('Forward X-direction [m]');
ylabel('Vertical Z-direction [m]');
title('Foot Trajectory Animation (Trot Gait)', 'FontSize', 15 );

% Define plots for the legs
rear_right_plot = plot(0, 0, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
front_right_plot = plot(0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
trajectory_line_rl = plot(rear_rigth_traj(1, :), rear_rigth_traj(2, :), 'g--');
trajectory_line_fr = plot(front_right_traj(1, :), front_right_traj(2, :), 'r--');

% Add legend
legend([rear_right_plot, front_right_plot], 'Rear Right Leg', 'Front Right Leg', 'Location', 'best');

% Animate foot trajectories
for i = 1:num_steps
    % Update positions
    set(rear_right_plot, 'XData', rear_rigth_traj(1, i), 'YData', rear_rigth_traj(2, i));
    set(front_right_plot, 'XData', front_right_traj(1, i), 'YData', front_right_traj(2, i));
    
    % Pause to simulate real-time motion
    pause(T0 / num_steps);
end

hold off;

