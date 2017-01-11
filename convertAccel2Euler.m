function [phi, theta] = convertAccel2Euler(ax, ay, az)

g = 9.8;

%theta = asin(ax/g);
theta = atan(-ax/sqrt(ay^2+az^2));
%phi   = asin(-ay / (g*cos(theta)) );
phi   = atan2(ay, az);
