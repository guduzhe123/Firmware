function [ out ] = gradual(value,x_low,x_high,y_low,y_high)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

if (value < x_low)
    out =y_low;
elseif (value > x_high)
    out = y_high;
else
    %linear function between the two points */
    a = (y_high - y_low) / (x_high - x_low);
    b = y_low - a * x_low;
    out =  a * value + b;
end

end