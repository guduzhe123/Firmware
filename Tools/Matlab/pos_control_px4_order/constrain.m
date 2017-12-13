function [ out ] = constrain( i, min, max)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
if i > max
    i = max;
end

if i < min
    i = min;
end

out = i;

end