%% Initialization
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

%% Show gripper fingers' move
fingerAngleTrajectory1 = linspace(pi/9, -pi/4, 300);
fingerAngleTrajectory2 = linspace(-pi/4, pi/9, 300);
fingerAngleTrajectory = [fingerAngleTrajectory1, fingerAngleTrajectory2];

% Plot result
for i = 1:600
    config = robot.homeConfiguration;
    gripperAngle = fingerAngleTrajectory(i) * [1 -1]';
    config(8:9,:) = gripperAngle;
    
    show(robot, config, 'PreservePlot', false, 'FastUpdate', true);
    axis padded
    drawnow;
end