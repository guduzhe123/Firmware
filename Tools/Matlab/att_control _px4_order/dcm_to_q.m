function [ Q ] = dcm_to_q( R )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
tr = R(1,1)+ R(2,2) + R(3,3);
Q = zeros(1,4);
if tr > 0.0
    s = sqrt(tr + 1.0);
	Q(1) = s * 0.5;
	s = 0.5 / s;
	Q(2) = (R(3,2) - R(2,3)) * s;
	Q(3) = (R(1,3) - R(3,1)) * s;
	Q(4) = (R(2,1) - R(1,2)) * s;

else
% Find maximum diagonal element in dcm
%store index in dcm_i
	dcm_i = 1;
    for i = 2:3
        if R(i,i) > R(dcm_i,dcm_i)
			dcm_i = i;
        end
    end

	dcm_j = mod((dcm_i + 0), 3)+1;
	dcm_k = mod((dcm_i + 1), 3)+1;
	s = sqrt((R(dcm_i,dcm_i) - R(dcm_j,dcm_j) -R(dcm_k,dcm_k)) + 1.0);
	Q(dcm_i + 1) = s * 0.5;
	s = 0.5 / s;
	Q(dcm_j + 1) = (R(dcm_i,dcm_j) + R(dcm_j,dcm_i)) * s;
	Q(dcm_k + 1) = (R(dcm_k,dcm_i)+ R(dcm_i,dcm_k)) * s;
	Q(1) = (R(dcm_k,dcm_j) - R(dcm_j,dcm_k)) * s;
end

end
