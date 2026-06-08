function [bool] = PosCheck(x)
%POSCHECK checks if the inputted SP position is within limits
%   if bool = 0, Position is within limits
%   if bool = 1, Position is outside limits
LA = SPConfig(x);
bool = ActCheck(LA);
end

