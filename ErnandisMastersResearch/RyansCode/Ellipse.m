function ellipse(a,b,X,Y)
%ELLIPSE: returns an array of points to plot an ellipse centered at x and y with a radius r
t = linspace(0,2*pi);
x = a*cos(t)+X;
y = b*sin(t)+Y;
end

