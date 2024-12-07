clc; clear;
L1 = 38.55;
L2 = 120;
L3 = 187.75;

r = robot(L1, L2, L3);
x = (L2+L3)/sqrt(2);
X = [x,x,L1];
IK_output = r.IK(X(1), X(2), X(3));
if (IK_output ~= [-1, -1, -1])
    IK_output = deg2rad(IK_output);
    % Define Link Lengths
    L1 = 38.55; % Length of Link 1
    L2 = 120; % Length of Link 2
    L3 = 187.75; % Length of Link 3
    % Create SerialLink Robot using DH Parameters
    L(1) = Link('revolute', 'd', L1, 'a', 0, 'alpha', pi/2);
    L(2) = Link('revolute', 'd', 0, 'a', L2, 'alpha', 0);
    L(3) = Link('revolute', 'd', 0, 'a', L3, 'alpha', 0);
    r = SerialLink(L, 'name', '3DOF_Robot');
    
    disp("Input points into the IK algorthim are:")
    disp(X)
    
    disp("Output of IK algorithm:")
    disp(IK_output)
    
    A = double(r.A([1,2,3], IK_output));
    pts = A(1:3, 4);

    disp("Using the FK algorithm from the robotics systems toolbox, we can verify that the robot would reach the end point:")
    disp(pts')
end