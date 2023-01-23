function res = workspaceVisualizer(robot)
    N=5000;
    t1=210+(210-(-30)*rand(N,1));
    t2=210+(210-(-30)*rand(N,1));
    t3=210+(210-(-30)*rand(N,1));
    
    b1=-180+(-180-180)*rand(N,1);
    b2=-180+(-180-180)*rand(N,1);
    b3=-180+(-180-180)*rand(N,1);
    l1=0.360;l2=0.420;l3=0.400;l4=0.276;
    
    for i=1:N
        H1=MakeTransform(t1(i),0.36,0,-pi/2);
        H2=MakeTransform(t2(i),0,0,pi/2);
        H3=MakeTransform(t3(i),0.42,0,pi/2);
        H4=MakeTransform(b1(i),0,0,-pi/2);
        H5=MakeTransform(b2(i),0.4,0,-pi/2);
        H6=MakeTransform(b3(i),0.300,0,0);
    
        F=H1*H2*H3*H4*H5*H6;
        X=F(1,4);
        Y=F(2,4);
        Z=F(3,4);
        plot3(X,Y,Z,'.');
        hold on;
    end
    show(robot);
end