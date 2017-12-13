function [ R ] = q_to_dcm( q )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
R = zeros(3,3);
aSq = q(1)^2;
bSq = q(2)^2;
cSq = q(3)^2;
dSq = q(4)^2;
R(1,1) = aSq + bSq - cSq - dSq;
R(1,2) = 2.0 * (q(2) * q(3) - q(1) * q(4));
R(1,3) = 2.0 * (q(1) * q(3) + q(2) * q(4));
R(2,1) = 2.0 * (q(2) * q(3) + q(1) * q(4));
R(2,2) = aSq - bSq + cSq - dSq;
R(2,3) = 2.0 * (q(3) * q(4) - q(1) * q(2));
R(3,1) = 2.0 * (q(2) * q(4) - q(1) * q(3));
R(3,2) = 2.0 * (q(1) * q(2) + q(3) * q(4));
R(3,3) = aSq - bSq - cSq + dSq;
end