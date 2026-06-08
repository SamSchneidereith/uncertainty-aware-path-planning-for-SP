function [Len] = TrajLength(Trajectory)
%TRAJLENGTH: Returns the total length of the 6 DoF Trajectory
N = size(Trajectory,2);
Len = 0;
for i = 1:N-1
    % Calculate the trajectory length between each point
    Loc = norm(Trajectory(i,1:3)-Trajectory(i+1,1:3))+norm(rad2deg(Trajectory(i,4:6)-Trajectory(i+1,4:6)));
    Len = Len + Loc;
end

