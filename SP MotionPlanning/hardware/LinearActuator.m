classdef LinearActuator < handle
properties (Constant)
    strokeLength = 200 % mm
    numSteps = 1024 
    stepSize = LinearActuator.strokeLength/LinearActuator.numSteps % mm

    max_vel  = 3.5 % mm/s
    % repeatability = 0.8 % mm
    % mechanical Backlash = 0.3 % mm
    % -> mu = 0mm, sigma = 0.2904 % mm
    sigma = 0.2904;

    % From Ryan's code, physical constants - Sam
    BaseAdd = 56.9636;
    FrontAdd = 46.45+4.4000001;
    BaseAct = 242.4;
    ABSmin_stroke = LinearActuator.BaseAdd+LinearActuator.FrontAdd+LinearActuator.BaseAct;
    ABSmax_stroke = LinearActuator.ABSmin_stroke+200;
    
    min_stroke = LinearActuator.ABSmin_stroke+11*LinearActuator.stepSize;
    max_stroke = LinearActuator.ABSmin_stroke+1019*LinearActuator.stepSize;
end

methods (Static)
    function bool = actCheck(l_actuators)
        % ActCheck checks if the actuators are within set limits
        % Outputs a bool, true if within limits, false if out of limits

        bool = all(l_actuators >= LinearActuator.min_stroke & l_actuators <= LinearActuator.max_stroke);
    end

    function [inputs] = actConverter(l_actuators)
        % Rounds required leg lengths to nearest step
        inputs = zeros(size(l_actuators));
        inputs(:) = floor((l_actuators(:) - LinearActuator.ABSmin_stroke)/LinearActuator.stepSize);
    end
end
end
