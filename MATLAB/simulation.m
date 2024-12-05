clc;
clear;

% Define Link Lengths (use your actual values here)
L1 = 38.55;
L2 = 120;
L3 = 187.75;

% Create SerialLink Robot using DH Parameters
L(1) = Link('revolute', 'd', L1, 'a', 0, 'alpha', pi/2);
L(2) = Link('revolute', 'd', 0, 'a', L2, 'alpha', 0);
L(3) = Link('revolute', 'd', 0, 'a', L3, 'alpha', 0);
robot = SerialLink(L, 'name', '3DOF_Robot');
% Define sample joint angles (in radians)
theta = [pi/4, pi/6, pi/3];

% Compute Forward Kinematics (FK)
T = robot.fkine(theta);

% Display end-effector position
disp('End-Effector Position (x, y, z):');
disp(T.t);

% Visualize Robot Configuration
figure;
robot.plot(theta);  % Visualize robot at the specified joint angles
title('3DOF Robot Forward Kinematics Simulation');
% Define target position (end-effector position in Cartesian coordinates)
target_pose = transl(200, 100, 50);  % we can  Modify the target position as needed

% Solve Inverse Kinematics
theta_ik = robot.ikine(target_pose, 'mask', [1 1 1 0 0 0]); % Solving for position only

% Display joint angles
disp('Calculated Joint Angles (IK):');
disp(theta_ik);

% Visualize robot at the calculated joint angles
figure;
robot.plot(theta_ik);  % Visualize robot at IK solution
title('3DOF Robot Inverse Kinematics Simulation');

% Define Start and End Joint Angles
theta_start = [0, 0, 0];          % Starting joint angles (radians)
theta_end = [pi/4, pi/6, pi/3];   % End joint angles (radians)

% Generate Joint Trajectory
steps = 50; % Number of steps in the trajectory
theta_trajectory = jtraj(theta_start, theta_end, steps);

% Simulate Robot Movement
figure;
robot.plot(theta_trajectory);
title('3DOF Robot Trajectory Simulation');

% Define start and end end-effector positions (using transl)
start_pose = transl(100, 50, 50);  % Starting end-effector position
end_pose = transl(200, 100, 75);  % End end-effector position

% Solve inverse kinematics for start and end poses
theta_start = robot.ikine(start_pose, 'mask', [1 1 1 0 0 0]);
theta_end = robot.ikine(end_pose, 'mask', [1 1 1 0 0 0]);

% Generate trajectory (interpolate joint angles between start and end)
theta_trajectory = jtraj(theta_start, theta_end, 50);  % 50 steps in the trajectory

% Simulate robot movement along the trajectory
figure;
robot.plot(theta_trajectory);
title('3DOF Robot Cartesian Trajectory Simulation');

% Define joint angle ranges
theta1_range = 0:0.1:pi;  % Joint 1 range
theta2_range = -pi/2:0.1:pi/2;  % Joint 2 range
theta3_range = -pi/2:0.1:pi/2;  % Joint 3 range

% Initialize workspace points
workspace = [];

% Loop through all combinations of joint angles
for theta1 = theta1_range
    for theta2 = theta2_range
        for theta3 = theta3_range
            T = robot.fkine([theta1, theta2, theta3]);  % Forward Kinematics
            position = T.t;  % Extract end-effector position
            workspace = [workspace; position'];  % Store position in workspace
        end
    end
end

% Plot the workspace
figure;
plot3(workspace(:,1), workspace(:,2), workspace(:,3), 'r.', 'MarkerSize', 4);
grid on;
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('3-DOF Robotic Arm Workspace');
axis equal;