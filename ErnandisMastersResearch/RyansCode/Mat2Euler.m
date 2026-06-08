function [vec] = Mat2Euler(R)
%Rotation Matrix to Euler for RzRyRz 
% vec = [RotX, RotY, RotZ]
vec(1) = atan(R(3,2)/R(3,3));
vec(2) = atan(-R(3,1)/sqrt(R(1,1)^2+R(2,1)^2));
vec(3) = atan(R(2,1)/R(1,1));
vec = vec';
end

