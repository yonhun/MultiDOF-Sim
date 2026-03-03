%% Intialization
clear all;
close all;
clc;

%% Load robot model
robot = importrobot('iiwa14.urdf');
robot.DataFormat = 'column';

%% Create and add the gripper to the robot
% Create a new rigid body for the gripper palm
gripperPalm = rigidBody('gripperPalm');

gripperPalmJoint = rigidBodyJoint('gripperPalmJoint', 'fixed');

setFixedTransform(gripperPalmJoint, trvec2tform([0, 0, 0.005]));
gripperPalm.Joint = gripperPalmJoint;

% add geometry to gripper palm(Box)
palmSize = [0.08, 0.08, 0.01];
gripperPalmGeometry = collisionBox(palmSize(1), palmSize(2), palmSize(3));
addCollision(gripperPalm, gripperPalmGeometry);
addVisual(gripperPalm, 'Box', palmSize);

addBody(robot, gripperPalm, 'iiwa_link_ee_kuka');

% Create and add the gripper finger's joint(revolute)
for i = 1:2
    gripperFingerJoint = rigidBody(['gripperFingerJoint', num2str(i)]);    
    gripperFingerJointJoint = rigidBodyJoint(['gripperFingerJointJoint', num2str(i)], 'revolute');
    
    % rotation matrix to set Joint's Z-axis to match Global X-axis 
    rotm = [0 0 1;
            1 0 0; 
            0 1 0];
    
    if i == 1
        setFixedTransform(gripperFingerJointJoint, trvec2tform([0, 0.045, 0]) * rotm2tform(rotm)); % Finger Joint 1 position
    else
        setFixedTransform(gripperFingerJointJoint, trvec2tform([0, -0.045, 0]) * rotm2tform(rotm)); % Finger Joint 2 position
    end
    
    gripperFingerJoint.Joint = gripperFingerJointJoint;

    % add geometry to gripper fingers' joint(Cylinder)
    fingerJointSize = [0.005, 0.08]; 
    gripperFingerGeometry = collisionCylinder(fingerJointSize(1), fingerJointSize(2));
    addCollision(gripperFingerJoint, gripperFingerGeometry);
    addVisual(gripperFingerJoint, 'Cylinder', fingerJointSize);
    
    addBody(robot, gripperFingerJoint, 'gripperPalm');
end

% Create and add the gripper finger body
for i = 1:2
    gripperFingerBody = rigidBody(['gripperFingerBody', num2str(i)]);
    gripperFingerBodyJoint = rigidBodyJoint(['gripperFingerBodyJoint', num2str(i)], 'fixed');
    
    setFixedTransform(gripperFingerBodyJoint, trvec2tform([0, 0.05, 0])); % Finger Body position

    gripperFingerBody.Joint = gripperFingerBodyJoint;
    
    % add geometry to gripper fingers' body(Box)
    fingerBodySize = [0.01, 0.09, 0.08]; 
    gripperFingerBodyGeometry = collisionBox(fingerBodySize(1), fingerBodySize(2), fingerBodySize(3));
    addCollision(gripperFingerBody, gripperFingerBodyGeometry);
    addVisual(gripperFingerBody, 'Box', fingerBodySize);
    
    if i == 1
        addBody(robot, gripperFingerBody, 'gripperFingerJoint1');
    else
        addBody(robot, gripperFingerBody, 'gripperFingerJoint2');
    end
end

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
initialguess = [0 0 0 0 0 0 0 -pi/4 pi/4]'; 

%% Trajectory Planning(Parameter S)
q0 = 0;
qf = 1;
tf = input('Final Time = ');
method = input('methods = ');

t = [0:0.01:tf];

% Cubic polinomial trajectory
if method == 1
    s = q0 + 3/tf^2*(qf-q0)*t.^2 - 2/tf^3*(qf-q0)*t.^3;
    sd = 6/tf^2*(qf-q0)*t - 6/tf^3*(qf-q0)*t.^2;
    sdd = 6/tf^2*(qf-q0) - 12/tf^3*(qf-q0)*t;
end

% Quintic polinomial trajectory
if method == 2
    s = q0 + 10*(qf-q0)/tf^3*t.^3 - 15*(qf-q0)/tf^4*t.^4 + 6*(qf-q0)/tf^5*t.^5;
    sd = 3*10*(qf-q0)/tf^3*t.^2 - 4*15*(qf-q0)/tf^4*t.^3 + 5*6*(qf-q0)/tf^5*t.^4;
    sdd = 6*10*(qf-q0)/tf^3*t - 12*15*(qf-q0)/tf^4*t.^2 + 20*6*(qf-q0)/tf^5*t.^3;
end

% LSPB trajectory
if method == 3

    a = input('acceleration = ');

    tb = tf/2 - (sqrt(a^2*tf^2 - 4*a*(abs(qf)-abs(q0))))/(2*a);
    if qf >= 0
        V = a*tb;
    else
        V = -a*tb;
        a = -a;
    end
    
    % Segment 1
    t1 = [0:0.01:tb];
    q1 = q0 + V/(2*tb)*t1.^2;
    dq1 = a*t1;
    ddq1 = a*ones(1,length(t1));
    
    % Segment 2
    t2 = [tb+0.01:0.01:tf-tb];
    q2 = (qf + q0 - V*tf)/2 + V*t2;
    dq2 = V*ones(1, length(t2));
    ddq2 = 0*ones(1,length(t2));
    
    % Segment 3
    t3 = [tf-tb+0.01:0.01:tf];
    q3 = qf - a/2*tf^2 + a*tf*t3 - a/2*t3.^2;
    dq3 = a*tf - a*t3;
    ddq3 = -a*ones(1,length(t3));
    
    % Full trajectory
    t = [t1 t2 t3];
    s = [q1 q2 q3];
    sd = [dq1 dq2 dq3];
    sdd = [ddq1 ddq2 ddq3];
end
%% Cartesian Space Trajectory Planning(Line)
for i = 1:length(s)
    % Calculate position along the line
    p_s = p_i + s(i) * v_xy;
    end_effector_pose(1:3, i) = p_s;
    
    % Calculate orientation
    r = [0 0 1];
    th_if = pi/2;
    th_s = s(i) * th_if;
    
    axang = [r th_s];
    R_s7 = o_i7 * axang2rotm(axang);
    o_s = R_s7 * [0 0 -1;
                  0 1  0;
                  1 0  0];

    % Make desired transform matrix
    T_sd(1:3,4) = p_s;
    T_sd(1:3,1:3) = o_s;
    T_sd(4,1:4) = [0 0 0 1];

    % Inverse Kinematics with points
    [configsol, ~] = ik('iiwa_link_ee', T_sd, weights, initialguess);

    config(1:9, i) = configsol(1:9);
    theta(1:7, i) = config(1:7, i);

    initialguess = configsol; % update initialguess to Prev_sol

end

%% Plot results
% Plot Parameter S
figure(1)
subplot(311);
hold on;
plot(t,s);
title('Trajectory planning')
hold off;
ylabel('S');
subplot(312)
hold on;
plot(t,sd);
hold off;
ylabel('Sd');
subplot(313);
hold on;
plot(t,sdd);
hold off;
ylabel('Sdd');
xlabel('Time(s)');

% Plot End-effector Position change
figure(2)
subplot(3,1,1);
plot(t, end_effector_pose(1,:));
title('End-effector Position')
ylabel('X Position(m)');
subplot(3,1,2);
plot(t, end_effector_pose(2,:));
ylabel('Y Position(m)');
subplot(3,1,3);
plot(t, end_effector_pose(3,:));
xlabel('Time(s)');
ylabel('Z Position(m)');

% Plot Joint Angle change
figure(3)
plot(t, theta);
title('Joint Angle Change');
xlabel('Time(s)');
ylabel('Joint Angle(rad)')
legend('\theta_1', '\theta_2', '\theta_3', '\theta_4', '\theta_5', '\theta_6', '\theta_7');

%% Animate Robot's move
% Plot Robot Trajectory
figure(4)
show(robot, config(:,1), 'PreservePlot', false); 
hold on;
axis padded;
plot3(end_effector_pose(1,:), end_effector_pose(2,:), end_effector_pose(3,:), 'r-', 'LineWidth', 2);

% Show Robot's Move
for i = 1:length(s)
    targetJointPosition = config(:, i); 
    figure(4)
    show(robot, targetJointPosition, 'PreservePlot', false, 'FastUpdate', true); 
    drawnow;
end

% Show Gripper's move
fingerAngleTrajectory = linspace(-pi/4, pi/9, 300); % Define Finger Trajectory(pi/4 ~ -pi/9)

for i = 1:300
    config_g = initialguess; % Set Configuration to Last Configuration
    gripperAngle = fingerAngleTrajectory(i) * [1 -1]';
    config_g(8:9,:) = gripperAngle;
    
    show(robot, config_g, 'PreservePlot', false, 'FastUpdate', true);
    drawnow;
end

hold off