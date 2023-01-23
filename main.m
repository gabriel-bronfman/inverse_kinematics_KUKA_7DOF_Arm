close all;

%% Load Prereq Display Information

robot = importrobot('iiwa14.urdf');

%% Forward Kinematics

% Forward Kinematics has two different functions, one to calculate the Tbe
% and one to dislay the position desired by specifying joint angles

desJointsAngles = [0 0 0 0 0 0 0];

% This is a Tbe, given my own DH table and code for the end-effector
calculated = forwardKinematicsCalculation(desJointsAngles);

% This function simply tells the URDF file how to orient itself
config = forwardKinematicsDisplay(robot,desJointsAngles);

% Create visual for given Forward Kinematics computation
figure
show(robot, config);

% Can decide to use these lines to confirm that calculated EE position is
% exactly the same as where the actual EE is in the visual

% getTransform(robot,config,'iiwa_link_ee')
% calculated

%% Inverse Kinematics

% Input desired angles here
desEEPos = [0; 0; pi/2; .3; .3; .3;];

% Function that actually computes IK and then sets a configuration object
% for the robot equal to the computed joint angles
jointAngles = inverseKinematics(desEEPos)
config = forwardKinematicsDisplay(robot,jointAngles);

% Performance calculation - Calculates the true target transformation (i.e
% what the actual Tbe should be given [alpha beta gamma x y z]) and the
% calculated transformation given computed joint angles, then finds the
% difference per term to determine accuracy 
trueTargetTransformation = makeTransformWithDesiredOrientation(desEEPos);
calculatedTransformation1 = MakeTransformOfEEinB([jointAngles 0]);
errorPerTerm = trueTargetTransformation-calculatedTransformation1

% Uses previously defined Config object to show robot in 3-space 
figure
show(robot,config);

%% Inverse Kinematics using Jacobian

%This seciton uses a gradient descent approach to estimate kinematics
%within about 5% accuracy and then displays the new pose of the robot
desiredPos = [pi/4; 0; pi/2; .3; .3; .3;];

[jacobianConfig, jointAngles] = InverseKinematicsUsingJacobian(robot, desiredPos)

figure
show(robot,jacobianConfig);

trueTargetTransformation = makeTransformWithDesiredOrientation(desiredPos);
calculatedTransformation2 = MakeTransformOfEEinB([jointAngles; 0])
errorPerTerm = trueTargetTransformation-calculatedTransformation2

%% Workspace Visualization

workspaceVisualizer(robot);
