function [out] = wrap_pi(bearing)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
	M_PI_F = 3.14159265;
	M_TWOPI_F = 6.28318531;
    while (bearing >= M_PI_F)
        bearing = bearing - M_TWOPI_F;
    end
    while (bearing < -M_PI_F)
		bearing = bearing + M_TWOPI_F;
    end
    out = bearing;
end