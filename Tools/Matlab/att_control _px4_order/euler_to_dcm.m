function [ R ] = euler_to_dcm( angle )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
R = zeros(3);
roll = angle(1);
pitch = angle(2);
yaw = angle(3);

cp = cos(pitch);
sp = sin(pitch);
sr = sin(roll);
cr = cos(roll);
sy = sin(yaw);
cy = cos(yaw);

R(1,1) = cp * cy;
R(1,2) = (sr * sp * cy) - (cr * sy);
R(1,3) = (cr * sp * cy) + (sr * sy);
R(2,1) = cp * sy;
R(2,2) = (sr * sp * sy) + (cr * cy);
R(2,3) = (cr * sp * sy) - (sr * cy);
R(3,1) = -sp;
R(3,2) = sr * cp;
R(3,3) = cr * cp;

end