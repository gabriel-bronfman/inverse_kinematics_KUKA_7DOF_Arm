function result = inverseKinematics(pose)
    % Define symbolic representation of EE transformation in Base
    % subsitute 0 for the EE joint so that we have less to worry about
    syms theta1 theta2 theta3 theta4 theta5 theta6 theta7;
    alpha = pose(1); beta = pose(2); gamma = pose(3); x = pose(4); y = pose(5); z = pose(6);
    dhTable = [theta1 .36 0 -pi/2;
           theta2 0 0 pi/2;
           theta3   .42 0 pi/2;
           theta4   0   0 -pi/2;
           theta5   .4  0 -pi/2;
           theta6   0   0  pi/2;
           theta7   .126 0  0];
    Tbe = MakeTransformOfEEinB(dhTable);
    Tbe = subs(Tbe,theta7,0);
    
    if sqrt(x^2 + y^2 + z^2) > 1.36
        result = [0 0 0 0 0 0];
        msg = 'You requested a position outside the workspace of the robot';
        error(msg);
    end
    result = [0 0 0 0 0 0];
     F = [beta  == atan2(-Tbe(3,1),sqrt(Tbe(2,1)^2 + Tbe(1,1)^2));
         alpha == atan2(Tbe(2,1)/cos(beta),Tbe(1,1)/cos(beta));
         gamma == atan2(Tbe(3,2)/cos(beta),Tbe(3,3)/cos(beta));
         x     == Tbe(1,4);
         y     == Tbe(2,4);
         z     == Tbe(3,4)];
     digits(5)
      while (result(2) < pi/2 || result(2) > 3*pi/2) || (result(4) < pi/2 || result(4) > 3*pi/2) 
        sol = vpasolve(F,'Random', true);
        result = [mod(double(sol.theta1),2*pi) mod(double(sol.theta2),2*pi) mod(double(sol.theta3),2*pi) mod(double(sol.theta4),2*pi) mod(double(sol.theta5),2*pi) mod(double(sol.theta6),2*pi)];  
      end
    
end