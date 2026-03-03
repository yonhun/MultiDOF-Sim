%% Initialization
clear all;
close all;
clc;

%% D-H Parameter Setting
theta = input('Enter the theta list: ');

d = [0.36 0 0.42 0 0.4 0 0.126];
alpha = [-pi/2 pi/2 pi/2 -pi/2 -pi/2 pi/2 0];
a = [0 0 0 0 0 0 0];

%% Calculate Transformation Matrix
T = eye(4);

for i = 1:7
    A = [cos(theta(i)), -sin(theta(i))*cos(alpha(i)), sin(theta(i))*sin(alpha(i)), a(i)*cos(theta(i));
         sin(theta(i)), cos(theta(i))*cos(alpha(i)), -cos(theta(i))*sin(alpha(i)), a(i)*sin(theta(i));
         0, sin(alpha(i)), cos(alpha(i)), d(i);
         0, 0, 0, 1];

    T = T * A;
end

result = T(1:3, 4)';
%% Print Position of End-Effector
disp(result);