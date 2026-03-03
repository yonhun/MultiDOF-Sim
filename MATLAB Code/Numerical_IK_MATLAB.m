%% Initialization
clear all; 
close all; 
clc;

%% Load the KUKA iiwa 14 R820 robot model
robot = importrobot('iiwa14.urdf');
robot.DataFormat = 'column';

%% Setting Target Pose
randConfig = robot.randomConfiguration;
T_sd = getTransform(robot,randConfig,'iiwa_link_ee');
targetPosition = T_sd(1:3, 4);

%% Setting Inverse Kinematics Solver
ik = inverseKinematics('RigidBodyTree', robot);
weights = [1 1 1 1 1 1];
initialguess = robot.homeConfiguration;

%% Solve Inverse Kinematics
[configSol, ~] = ik('iiwa_link_ee', T_sd, weights, initialguess);

%% Forward Kinematics with Solution
theta = configSol;
d = [0.36 0 0.42 0 0.4 0 0.126];
alpha = [-pi/2 pi/2 pi/2 -pi/2 -pi/2 pi/2 0];
a = [0 0 0 0 0 0 0];

T = eye(4);

for i = 1:7
    A = [cos(theta(i)), -sin(theta(i))*cos(alpha(i)), sin(theta(i))*sin(alpha(i)), a(i)*cos(theta(i));
         sin(theta(i)), cos(theta(i))*cos(alpha(i)), -cos(theta(i))*sin(alpha(i)), a(i)*sin(theta(i));
         0, sin(alpha(i)), cos(alpha(i)), d(i);
         0, 0, 0, 1];

    T = T * A;
end

result = T(1:3, 4);

%% Plot results
disp('Target Position: ');
disp(targetPosition');
disp('Inverse Kinematics Solution:');
disp(configSol');
disp('Position Calculated with IK Solution: ');
disp(result');