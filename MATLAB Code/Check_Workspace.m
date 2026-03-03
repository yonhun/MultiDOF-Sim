%% Initialization
clear all;
close all;
clc;

%% D-H Parameter Setting

num_points = 100;
theta_mat = linspace(-pi*(2/3), pi*(2/3), num_points); % devide (-120 ~ 120)(deg) to 100 points  
d = [0.36 0 0.42 0 0.4 0 0.126];
alpha = [-pi/2 pi/2 pi/2 -pi/2 -pi/2 pi/2 0];
a = [0 0 0 0 0 0 0];

%% Calculate Transformation Matrix
ws_x = []; % Workspace's x pose 
ws_z = []; % Workspace's z pose

for i1 = 1:num_points
    for i2 = 1:num_points
        for i3 = 1:num_points
            % Initialize Transformation Matrix
            T = eye(4);
            
            theta = [0 theta_mat(i1) 0 theta_mat(i2) 0 theta_mat(i3) 0];
            for i = 1:7
                A = [cos(theta(i)), -sin(theta(i))*cos(alpha(i)), sin(theta(i))*sin(alpha(i)), a(i)*cos(theta(i));
                     sin(theta(i)), cos(theta(i))*cos(alpha(i)), -cos(theta(i))*sin(alpha(i)), a(i)*sin(theta(i));
                     0, sin(alpha(i)), cos(alpha(i)), d(i);
                     0, 0, 0, 1];
    
                T = T * A;
            end
            
            % Store end-effector position
            ws_x(end+1) = T(1, 4);
            ws_z(end+1) = T(3, 4);
        end
    end
end

%% Plot Work Space
figure;
scatter(ws_x(:), ws_z(:), '.');
title('Work Space');
xlabel('X-axis');
ylabel('Z-axis');
