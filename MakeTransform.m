function TransMatrix = MakeTransform(theta,d,a,alpha)
    TransMatrix = [cos(theta), -1*cos(alpha)*sin(theta), sin(alpha)*sin(theta), a * cos(theta);
                   sin(theta), cos(alpha)*cos(theta), -1*sin(alpha) *cos(theta), a * sin(theta);
                   0,          sin(alpha),            cos(alpha), d;
                   0,           0,                      0,        1];
end