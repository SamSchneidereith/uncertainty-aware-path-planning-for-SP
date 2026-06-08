function [Xout] = MoveAct(Start,Goal,delta)
BaseAdd = 56.9636;               
FrontAdd = 46.45+4.4000001;
BaseAct = 242.4;
step = 200/1024;
ABSmin_stroke = BaseAdd+FrontAdd+BaseAct;   % Absolute min length of actuator
min_stroke = ABSmin_stroke+11*step;         % Preferred min length or actuator

M = [cosd(45) -sind(45); sind(45) cosd(45)];
% Find input:
D = M\(delta*(Normalize(Goal-Start)));
% Find New Position:
Xout = (M*(D))+Start;
end

