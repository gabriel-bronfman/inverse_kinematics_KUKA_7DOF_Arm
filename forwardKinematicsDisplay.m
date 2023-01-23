function config = forwardKinematicsDisplay(robot,jointAngles)
    config = homeConfiguration(robot);
    for index = 1:length(jointAngles)
        config(index).JointPosition = jointAngles(index);
    end
end