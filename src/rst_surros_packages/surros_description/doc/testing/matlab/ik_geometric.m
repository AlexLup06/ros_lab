function theta = ik_geometric(H,dh)
theta = nan(5,1);
R_tcp = H(1:3,1:3);
o_tcp = H(1:3,4);

o_3 = o_tcp - dh.d(6)*R_tcp*[0;0;1] - dh.a(6)*R_tcp*[1;0;0];
x_o3 = o_3(1);
y_o3 = o_3(2);
z_o3 = o_3(3);

r = sqrt(x_o3^2 + y_o3^2);
s = z_o3 - dh.d(1);
W = (r^2 + s^2 - dh.a(2)^2 - dh.a(3)^2)/(2*dh.a(2)*dh.a(3));

theta(1) = atan2(y_o3,x_o3);
theta(3) = atan2(sqrt(1-W^2),W);
theta(2) = -(atan2(s,r) - atan2(dh.a(3)*sin(-theta(3)),dh.a(2) + dh.a(3)*cos(-theta(3))) );

Y = cos(theta(1))*cos(theta(2)+theta(3))*R_tcp(1,3) + ...
    sin(theta(1))*cos(theta(2)+theta(3))*R_tcp(2,3) - ...
    sin(theta(2)+theta(3))*R_tcp(3,3);
X = cos(theta(1))*sin(theta(2)+theta(3))*R_tcp(1,3) + ...
    sin(theta(1))*sin(theta(2)+theta(3))*R_tcp(2,3) + ...
    cos(theta(2)+theta(3)) * R_tcp(3,3);

theta(4) = atan2(Y,X);

T_temp = eye(4,4);
theta(5) = 0;
%theta(6) = dh.theta(6);
dh.theta(1:5) = theta;
for i=1:4 %numel(theta)
    T_temp = T_temp * getDHmatrix(dh(i,:));
end
theta(5) = acos(T_temp(:,2)' * H(:,2)) - dh.theta(6);
theta = theta(1:5);
end