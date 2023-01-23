function result = makeTransformWithDesiredOrientation(pose)
    result = [cos(pose(1))*cos(pose(2)) cos(pose(1))*sin(pose(2))*sin(pose(3)) - sin(pose(1))*cos(pose(3)) cos(pose(1))*sin(pose(2))*cos(pose(3)) + sin(pose(1))*sin(pose(3)) pose(4);
              sin(pose(1))*cos(pose(2)) sin(pose(1))*sin(pose(2))*sin(pose(3)) + cos(pose(1))*cos(pose(3)) sin(pose(1))*sin(pose(2))*cos(pose(3)) - cos(pose(1))*sin(pose(3)) pose(5);
              -sin(pose(2))             cos(pose(2))*sin(pose(3))                                          cos(pose(2))*cos(pose(3))                                          pose(6);
              0                         0                                                                  0                                                                  1]
end