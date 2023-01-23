function Tbe = MakeTransformOfEEinB(angles)
    Tbe = eye(4);
    dhTable = [angles(1) .36 0 -pi/2;
           angles(2) 0 0 pi/2;
           angles(3)   .42 0 pi/2;
           angles(4)   0   0 -pi/2;
           angles(5)   .4  0 -pi/2;
           angles(6)   0   0  pi/2;
           angles(7)   .126 0  0];
    for index = 1:length(angles)
        
        Tbe = Tbe * MakeTransform(angles(index),dhTable(index,2),dhTable(index,3),dhTable(index,4));
    end
end