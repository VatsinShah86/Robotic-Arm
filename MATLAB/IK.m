clc; clear;
L1 = 38.55;
L2 = 120;
L3 = 187.75;

r = robot(L1, L2, L3);
x = L2+L3;
r.IK(0, 0, 0)