function T = getDHmatrix(dh)
    T = zeros(4,4);
    
    T(1,1) = cos(dh.theta);
    T(1,2) = -sin(dh.theta)*cos(dh.alpha);
    T(1,3) = sin(dh.theta)*sin(dh.alpha);
    T(1,4) = dh.a*cos(dh.theta);

    T(2,1) = sin(dh.theta);
    T(2,2) = cos(dh.theta)*cos(dh.alpha);
    T(2,3) = -cos(dh.theta)*sin(dh.alpha);
    T(2,4) = dh.a*sin(dh.theta);

    T(3,2) = sin(dh.alpha);
    T(3,3) = cos(dh.alpha);
    T(3,4) = dh.d;

    T(4,4) = 1;
end