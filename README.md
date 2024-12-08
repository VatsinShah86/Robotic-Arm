# Robotic-Arm

This is our final project for MAE 547 (Modeling and Control of Robots). We will explain how to run this code in this file. The code only requires running the python code in main.py. It does not require any inputs. It requires the libraries numpy and gpiozero.

The parameters of the robot are defined in main.py, in the init function for the robot class. The target coordinates are defined as pWx, pWy, and pWz in the main operating flow of the code.

The Matlab folder contains Matlab code we used for simulation and workspace calculations. We used the Robotics Toolbox from Peter Corke to help with some of the calculations and verify the outputs from our inverse kinematic code (running on Raspberry Pi). The same IK algorithm running on Raspberry Pi is implemented in Matlab as well (robot.m). 

Team Members:
Dilli Babu Kalluru
Sukhpreet Singh
Vatsin Shah
Vinamr Arya
Vishvajit Khinda
