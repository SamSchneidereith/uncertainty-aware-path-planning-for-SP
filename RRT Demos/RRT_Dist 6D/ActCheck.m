function [bool] = ActCheck(LA)
    % ActCheck checks if the actuators are within set limits
    % Outputs a bool, true if within limits, false if out of limits
    
    BaseAdd = 56.9636;
    FrontAdd = 46.45+4.4000001;
    BaseAct = 242.4;
    step = 200/1024;
    ABSmin_stroke = BaseAdd+FrontAdd+BaseAct;
    ABSmax_stroke = ABSmin_stroke+200;
    
    min_stroke = ABSmin_stroke+11*step;
    max_stroke = ABSmin_stroke+1019*step;
    
    for i = 1:6
        bool = true;
        if LA(i) > max_stroke
            bool = false;
            %error('Acutator %d exceeds max\n',i);
        elseif LA(i) < min_stroke
            bool = false;
            %error('Actuator %d below min\n',i);
        end
    end
end

