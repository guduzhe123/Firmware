function [ angle ] = dcm_to_euler( dcm )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
angle = zeros(1,3);
%q = zeros(1,4);
phi_val = atan2(dcm(3, 2), dcm(3, 3));
theta_val = asin(-dcm(3, 1));
psi_val = atan2(dcm(2, 1), dcm(1, 1));
pi = 3.1415926;

if (abs(theta_val - pi / 2) < 1.0e-3)
    phi_val = 0.0;
    psi_val = atan2(dcm(2, 3), dcm(1, 3));

elseif (abs(theta_val + pi / 2) < 1.0e-3)
    phi_val = 0.0;
    psi_val = atan2(-dcm(2, 3), -dcm(1, 3));
end

angle(1) = phi_val;
angle(2) = theta_val;
angle(3) = psi_val;
end