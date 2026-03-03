%% Initialization
clear all; 
close all;
clc;

%% Load the KUKA iiwa 14 R820 robot model
robot = importrobot('iiwa14.urdf');
robot.DataFormat = 'column';

%% Define the desired end-effector pose
randConfig = randomConfiguration(robot);
T_sd = getTransform(robot,randConfig,'iiwa_link_ee');
xd = T_sd(1:3, 4);

%% Parameters
tol = 1e-2;
max_iter = 100;
attempt = 1;
success = false;

%% Solve Inverse Kinematics
while ~success
    % Generate a new initial joint configuration
    initialConfig = randomConfiguration(robot);
    th = initialConfig;

    k = 1;
    flag = 1;

    while flag == 1 && k <= max_iter
        currentPose = getTransform(robot, th, 'iiwa_link_ee');
        current_position = currentPose(1:3,4);

        % Store results
        result(k,1) = k;
        result(k,2:8) = th' * 180/pi;
        result(k,9:11) = current_position';

        % Error
        e = xd - current_position;

            % Check if the error is within tolerance
            if norm(e) < tol
                flag = 0;
                success = true;
            end

        % Jacobian calculation
        J = geometricJacobian(robot, th, 'iiwa_link_ee');
        Jv = J(1:3,:);

        % Setting small values in Jacovian to zero
        for i = 1:size(Jv,1)
            for j = 1:size(Jv,2)
                if abs(Jv(i,j)) < 0.0001
                    Jv(i,j) = 0;
                end
            end
        end

        % Update Joint angle
        th_new = th + pinv(Jv,0.0001) * e;
        th = th_new;

        k = k + 1;
    end

    % If Ik failed, try with a new initial configuration 
    %if ~success
    %    disp(['Attempt ', num2str(attempt), ' failed. Trying a new initial configuration']);
    %    attempt = attempt + 1;
    %end
    
end

%% Plot results
if success
    figure;
    plot(result(1:k-1, 1), result(1:k-1, 2:8));
    xlabel('Iteration');
    ylabel('Joint Angles(deg)');
    legend('theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6', 'theta7');
    grid on;

    disp('Target Position: ');
    disp(xd');
    disp('Inverse Kinematics solution:');
    disp(result(k-1, 2:8));
    disp('End-Effector Position:');
    disp(result(k-1, 9:11));
else
    disp('Failed to find a solution within the maximum number of attempts. ');
end