function [pos, jointAngles] = InverseKinematicsUsingJacobian(robot, desiredPos)
 
 % Using desired pose and current pose, the function calculates error in
 % cartesian space, maps that error to joint space, then takes incremental
 % steps of K magnitude towards the target, all the while updating the jacobian and
 % adjusting direction of steps in joint space
 
 currentJoints = [.1 .1 .1 .1 .1 .1 0];
 currentPose = MakeTransformOfEEinB(currentJoints);
 
 beta = atan2(-currentPose(3,1),sqrt(currentPose(2,1)^2 + currentPose(1,1)^2));
 alpha = atan2(currentPose(2,1)/cos(beta),currentPose(1,1)/cos(beta));
 gamma = atan2(currentPose(3,2)/cos(beta),currentPose(3,3)/cos(beta));
 error = desiredPos - [alpha; beta; gamma; currentPose(1,4);currentPose(2,4);currentPose(3,4)];


    while (abs(error(1)) > .05 || abs(error(2)) > .05 || abs(error(3)) > .05 || abs(error(4)) > .05  || abs(error(5)) > .05 || abs(error(6)) > .05 )
        jacobian = geometricJacobian(robot,forwardKinematicsDisplay(robot,currentJoints),'iiwa_link_ee');
        currentJoints = currentJoints + pinv(jacobian)*error*.1;
        %currentJoints = mod(currentJoints,2*pi);
        currentPose = MakeTransformOfEEinB(currentJoints);

        beta = atan2(-currentPose(3,1),sqrt(currentPose(2,1)^2 + currentPose(1,1)^2));
        alpha = atan2(currentPose(2,1)/cos(beta),currentPose(1,1)/cos(beta));
        gamma = atan2(currentPose(3,2)/cos(beta),currentPose(3,3)/cos(beta));
        error = desiredPos - [alpha; beta; gamma; currentPose(1,4);currentPose(2,4);currentPose(3,4)]
    end
  pos = forwardKinematicsDisplay(robot, currentJoints);
  jointAngles = currentJoints(1:6,1);
end