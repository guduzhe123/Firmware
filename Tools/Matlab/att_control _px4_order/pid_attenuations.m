function [ pidAttenuationPerAxis ] = pid_attenuations( tpa_breakpoint, tpa_rate, thrust,def)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
tpa = 1.0 - tpa_rate * (abs(thrust) - tpa_breakpoint) / (1.0 - tpa_breakpoint);
tpa = max(def.TPA_RATE_LOWER_LIMIT,min(1,tpa));
pidAttenuationPerAxis = zeros(1,3);
pidAttenuationPerAxis(def.AXIS_INDEX_ROLL) = tpa;
pidAttenuationPerAxis(def.AXIS_INDEX_PITCH) = tpa;
pidAttenuationPerAxis(def.AXIS_INDEX_YAW) = 1.0;
end