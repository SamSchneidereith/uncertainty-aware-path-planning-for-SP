function [V] = VolEllipsoid2(PMat)
%   VolEllipsoid: Calculates volume of an N dimensional Ellipsoid.
% PMat must be a square matrix
n = 3;
eigenvalues = eig(PMat(1:3,1:3));
D = sqrt(eigenvalues);
V = prod(D)*(2/n)*((pi^(n/2))/(gamma(n/2)));
end