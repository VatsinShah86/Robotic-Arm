clear;
close;
clc;
% Define Link Lengths
L1 = 65;
L2 = 120;
L3 = 187.75;

% Create SerialLink Robot using DH Parameters
L(1) = Link('revolute', 'd', L1, 'a', 0, 'alpha', pi/2);
L(2) = Link('revolute', 'd', 0, 'a', L2, 'alpha', 0);
L(3) = Link('revolute', 'd', 0, 'a', L3, 'alpha', 0);
robot = SerialLink(L, 'name', '3DOF_Robot');

% Define Start and End Joint Angles
theta_start = [0, 0, 0];          % Starting joint angles (radians)
theta_end = [pi/2, pi/4, pi/5];   % End joint angles (radians)

% Generate Joint Trajectory
steps = 50; % Number of steps in the trajectory
theta_trajectory = jtraj(theta_start, theta_end, steps);

% Initialize variables to store trajectory points
trajectory_points = [];

% Create the figure for dynamic simulation
figure;
hold on;
grid on;
xlim([-50, 500]);
ylim([-50, 500]);
zlim([0, 500]);
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('3DOF Robot Trajectory Simulation with Dynamic Path');
robot.plot(theta_start);  % Plot initial robot configuration
path_plot = plot3(0, 0, 0, 'b-', 'LineWidth', 2);  % Initialize trajectory path plot


% Simulate robot movement and update the trajectory path dynamically
for i = 1:steps
    % Get current joint angles
    current_theta = theta_trajectory(i, :);
    
    % Update robot plot
    robot.plot(current_theta);
    
    % Compute forward kinematics for the current joint angles
    T = robot.fkine(current_theta);
    current_position = T.t';  % Extract Cartesian position
    
    % Append the current position to trajectory points
    trajectory_points = [trajectory_points; current_position];
    
    % Update the trajectory path dynamically
    set(path_plot, 'XData', trajectory_points(:,1), ...
                   'YData', trajectory_points(:,2), ...
                   'ZData', trajectory_points(:,3));
    
    pause(0.1);  % Pause to simulate real-time plotting
end

hold off;
