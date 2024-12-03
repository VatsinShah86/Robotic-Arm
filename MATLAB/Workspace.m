clc;
clear;
% Define Link Lengths
L1 = 38.55; % Length of Link 1
L2 = 120; % Length of Link 2
L3 = 187.75; % Length of Link 3
% Create SerialLink Robot using DH Parameters
L(1) = Link('revolute', 'd', L1, 'a', 0, 'alpha', pi/2);
L(2) = Link('revolute', 'd', 0, 'a', L2, 'alpha', 0);
L(3) = Link('revolute', 'd', 0, 'a', L3, 'alpha', 0);
robot = SerialLink(L, 'name', '3DOF_Robot');
% Define Joint Angle Ranges with Steps
theta1_min = 0; theta1_max = pi; theta1_step = 0.14; % Joint 1
theta2_min = -pi/2; theta2_max = pi/2; theta2_step = 0.14; % Joint 2
theta3_min = -pi/2; theta3_max = pi/2; theta3_step = 0.14 ; % Joint 3
% Generate Ranges Dynamically
theta1_range = theta1_min:theta1_step:theta1_max;
theta2_range = theta2_min:theta2_step:theta2_max;
theta3_range = theta3_min:theta3_step:theta3_max;
% Initialize Workspace Points
workspace = [];
% Nested Loops to Calculate Workspace
for theta1 = theta1_range
    for theta2 = theta2_range
        for theta3 = theta3_range
            % Forward Kinematics
            T = robot.fkine([theta1, theta2, theta3]);
            position = T.t; % Extract x, y, z from transformation matrix
            workspace = [workspace; position']; 
        end
    end
end

% Plot Workspace
figure;
plot3(workspace(:,1), workspace(:,2), workspace(:,3 ),'r.','MarkerSize',4);
grid on;
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('3-DOF Robotic Arm Workspace');
axis equal;