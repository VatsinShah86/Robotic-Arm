clc; clear;
% Define Link Lengths of the robot
L1 = 38.55;
L2 = 120;
L3 = 187.75;

% Create an object for the robot from robot.m
r = robot(L1, L2, L3);

% This is to help define a point that is within the limits of the robot's
% workspace.
x = (L2+L3)/sqrt(2)/1.1;

% Vector of point we are trying to reach
X = [x,x,-60];

% Call inverse kinematics function from the object robot
IK_output = r.IK(X(1), X(2), X(3));

% If the point is unreachable, the response would be [-1, -1, -1]. Thus, we
% only execute this next code if that is not the response.
if (IK_output ~= [-1, -1, -1])
    % Convert output from degrees to radians
    IK_output = deg2rad(IK_output);
    
    % Create SerialLink Robot using DH Parameters
    L(1) = Link('revolute', 'd', L1, 'a', 0, 'alpha', pi/2);
    L(2) = Link('revolute', 'd', 0, 'a', L2, 'alpha', 0);
    L(3) = Link('revolute', 'd', 0, 'a', L3, 'alpha', 0);
    r = SerialLink(L, 'name', '3DOF_Robot');
    
    disp("Input points into the IK algorthim are:")
    disp(X)
    
    disp("Output of IK algorithm:")
    disp(IK_output)
    
    % Find the position of the end effector using FK
    A = double(r.A([1,2,3], IK_output));
    pts = A(1:3, 4);
    
    disp("Using the FK algorithm from the robotics systems toolbox, we can verify that the robot would reach the end point:")
    disp(pts')
end