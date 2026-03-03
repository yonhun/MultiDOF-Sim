%% Initialization
clear all;
close all;
clc;

%% Load robot model
robot = importrobot('iiwa14.urdf');
robot.DataFormat = 'column';

%% Define Initial, Final point/orientation
p_i = [0.6 0.5 0.4]; % Initial point
p_f = [0.4 -0.5 0.1]; % Final point
v_xy = p_f - p_i; % Direction vector
diff_x = v_xy(1); 
diff_y = v_xy(2);

% Initial orientation
o_i7 = [-1  0  0; 
         0  1  0;
         0  0 -1];

% Final orientation
o_f7 = [0  1  0;
        1  0  0;
        0  0 -1];

%% Create I.K solver
ik = inverseKinematics('RigidBodyTree', robot);
weights = [1 1 1 1 1 1];
initialguess = [0 0 0 0 0 0 0]'; 

%% Calculate Points on Trajectory & Check Trajectory is Valid

num_points = 100;
for i = 1:num_points
    % Calculate Points on Trajectory
    p_t = p_i + (i/num_points) * v_xy;

    % Calculate orientation
    r = [0 0 1];
    th_if = pi/2;
    th_t = (i/num_points) * th_if;

    axang = [r th_t];
    R_t7 = o_i7 * axang2rotm(axang);
    o_t = R_t7 * [0 0 -1;
                  0 1  0;
                  1 0  0];

    % Make desired transform matrix
    T_sd(1:3, 1:3) = o_t;
    T_sd(1:3, 4) = p_t;
    T_sd(4, 1:4) = [0 0 0 1];

    % Inverse Kinematics to find joint angles
    [config, info] = ik('iiwa_link_ee', T_sd, weights, initialguess);

    % Check if the solution is valid
    if strcmp(info.Status, 'success') == 1      
        % Update the initial guess for the next iteration
        initialguess = config;
    else
        disp('Inverse Kinematics failed.');
        break;
    end
end

disp('Trajectory points checked successfully.');