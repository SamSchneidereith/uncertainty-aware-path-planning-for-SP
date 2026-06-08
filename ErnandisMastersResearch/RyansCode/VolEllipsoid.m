function [V] = VolEllipsoid(PMat)
%   VolEllipsoid: Calculates volume of an N dimensional Ellipsoid.

% PMat must be a square matrix
n = size(PMat,1);
eigenvalues = eig(PMat);
D = sqrt(eigenvalues);
V = prod(D)*(2/n)*((pi^(n/2))/(gamma(n/2)));
end