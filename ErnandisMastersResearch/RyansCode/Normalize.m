function out = Normalize(in)
% Normalize a vector:
% in: input vector of whatever length and magnitude
% out: normalized vector
if norm(in) == 0
    out = in;
    return;
end
out = in/norm(in);
end
