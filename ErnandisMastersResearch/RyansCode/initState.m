function [X,bool] = initState(D)
% Input Actuator lengths, output state for 2D case
BaseAdd = 56.9636;               
FrontAdd = 46.45+4.4000001;
BaseAct = 242.4;
step = 200/1024;
ABSmin_stroke = BaseAdd+FrontAdd+BaseAct;   % Absolute min length of actuator
min_stroke = ABSmin_stroke+11*step;         % Preferred min length or actuator
d1 = D(1);
d2 = D(2);
if d1 < 11*step
    % error('d1 too small')
    bool = 1;
elseif d1 > 1019*step
    % error('d1 too large')
    bool = 1;
elseif d2 < 11*step
    % error('d2 too small')
    bool = 1;
elseif d2 > 1019*step
    % error('d2 too large')
    bool = 1;
else
    bool = 0;
end
X = [cosd(45) -sind(45); sind(45) cosd(45)]*[d1+ABSmin_stroke;d2+ABSmin_stroke];
end