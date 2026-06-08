function [len] = TrajLength2D(X)
N = size(X,1);
if N == 1
    len = 0;
else
    len = 0;
    for i = 1:N-1
        loc = norm(X(i,:)-X(i+1,:));
        len = len+loc;
    end
end
