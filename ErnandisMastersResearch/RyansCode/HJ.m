function [H,Hall] = HJ(x,Hall,h)
%HJ returns the Measurement Jacobian
%   x: all the states up to and including the current predicted state
%   SP: SP parameters

% Other Parameters:
%h = 1e-1;              % addition during finite difference
J = zeros(6,6);
iter = size(Hall,3)+1;
for i = 1:6
    Z = zeros(6,1);
    Z(i) = h;
    LAp = SPConfig(x+Z);
    LAm = SPConfig(x-Z);
    for j = 1:6
        J(j,i) = (LAp(j)-LAm(j))/2*h;
    end
end
if isempty(Hall)
    H = J;
    Hall(:,:,1) = H;
else
    Hall(:,:,iter) = J;
    H = mean(Hall,3);
end

end

