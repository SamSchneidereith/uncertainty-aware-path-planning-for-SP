function [R] = RotMat(vec)
%RotMat: Create rotation matrix based on inputted x,y,z rotations in vector form (in RAD):
Rx = [1 0 0; 0 cos(vec(1)) -sin(vec(1)); 0 sin(vec(1)) cos(vec(1))];
Ry = [cos(vec(2)) 0 sin(vec(2)); 0 1 0; -sin(vec(2)) 0 cos(vec(2))];
Rz = [cos(vec(3)) -sin(vec(3)) 0; sin(vec(3)) cos(vec(3)) 0; 0 0 1];

R = Rz*Ry*Rx;
end

