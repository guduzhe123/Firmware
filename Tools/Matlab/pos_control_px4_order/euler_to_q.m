function [ Q ] = euler_to_q( A )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
Q = zeros(1,4);
cr = cos(A(1)/2);
sr = sin(A(1)/2);
cp = cos(A(2)/2);
sp = sin(A(2)/2);
cy = cos(A(3)/2);
sy = sin(A(3)/2);
Q(1) = cr*cp*cy+sr*sp*sy;
Q(2) = sr*cp*cy-cr*sp*sy;
Q(3) = cr*sp*cy+sr*cp*sy;
Q(4) = cr*cp*sy-sr*sp*cy;
end