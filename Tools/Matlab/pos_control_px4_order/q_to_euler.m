function [ angle ] = q_to_euler( Q )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
w = Q(1);
x = Q(2);
y = Q(3);
z = Q(4);
angle = zeros(1,3);
angle(1) = atan2(2*(w*x+y*z),(1-2*(x^2+y^2)));
angle(2) = asin(2*(w*y-z*x));
angle(3) = atan2(2*(w*z+x*y),(1-2*(y^2+z^2)));

end