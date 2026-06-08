function [out] = ActCheck(LA)
%ActCheck checks if the actuators are within set limits
%   outputs a bool, 0 if within limits, 1 if out of limits


BaseAdd = 56.9636;
FrontAdd = 46.45+4.4000001;
BaseAct = 242.4;
step = 200/1024;
ABSmin_stroke = BaseAdd+FrontAdd+BaseAct;
ABSmax_stroke = ABSmin_stroke+200;

min_stroke = ABSmin_stroke+11*step;
max_stroke = ABSmin_stroke+1019*step;

for i = 1:6
    out = 0;
    if LA(i) > max_stroke
        out = 1;
        %error('Acutator %d exceeds max\n',i);
    elseif LA(i) < min_stroke
        out = 1;
        %error('Actuator %d below min\n',i);
    end
end

