function [bool] = PosCheck2D(Pos)
%POSCHECK2D checks if the inputted position [X,Y] is capabable of being
%reached by the actuators. Physical aspects of actuators hard coded in
%here.
BaseAdd = 56.9636;               
FrontAdd = 46.45+4.4000001;
BaseAct = 242.4;
step = 200/1024;
ABSmin_stroke = BaseAdd+FrontAdd+BaseAct;   % Absolute min length of actuator
R = [cosd(45) -sind(45); sind(45) cosd(45)];

D = R\Pos - [ABSmin_stroke; ABSmin_stroke];
bool = 0;
if D(1) < 11*step || D(1) > 1019*step
    bool = 1;
elseif D(2) < 11*step || D(2) > 1019*step
    bool = 1;
end
end

