clc; clear;

% Define Link Lengths
L1 = 38.55; % Length of Link 1
L2 = 120; % Length of Link 2
L3 = 187.75; % Length of Link 3
% Create SerialLink Robot using DH Parameters
L(1) = Link('revolute', 'd', L1, 'a', 0, 'alpha', pi/2);
L(2) = Link('revolute', 'd', 0, 'a', L2, 'alpha', 0);
L(3) = Link('revolute', 'd', 0, 'a', L3, 'alpha', 0);
robot = SerialLink(L, 'name', '3DOF_Robot');

A = robot.A([1,2,3], [0,0,0])