function [Actinputs,LA_lengths] = ActInputsConverter(T)
    % ActInputsConverter takes the transform matrix for a SP and outputs the
    % actuator inputs 
    %   
    % ActInputsConverter takes the transform matrix for a SP and outputs the
    % actuator inputs 
    %   
    % Use original measurements to set up plate location:
    jointRadius = 184.9416; % Joint distance from center of plates
    jointSpacing = 63.5; % Joint pair spacing
    alpha = asin((jointSpacing/2)/jointRadius); % angle for Ball joints
    
    P_TPactsTPcen = zeros(6,3);
    P_TPactsSP4 = zeros(6,4);
    P_BPactsSP = zeros(6,3);
    LA_lengths = zeros(6,1);
    Actinputs = zeros(6,1);
    
    % Bottom Plate Joint coords: (All Z are 0 since bottom plate)
    P_BPactsSP(1,1) = jointRadius*cos(alpha);  % Joint 0 X
    P_BPactsSP(1,2) = jointRadius*sin(alpha);  % Joint 0 Y
    P_BPactsSP(2,1) = jointRadius*cos(-alpha);  % Joint 1 X
    P_BPactsSP(2,2) = jointRadius*sin(-alpha);  % Joint 1 Y
    
    for i = 1:2
        P_BPactsSP(2*i+1,:) = [cosd(-120*i) -sind(-120*i) 0; sind(-120*i) cosd(-120*i) 0; 0 0 1] * P_BPactsSP(1,:)'; % Joint 2,4
        P_BPactsSP(2*i+2,:) = [cosd(-120*i) -sind(-120*i) 0; sind(-120*i) cosd(-120*i) 0; 0 0 1] * P_BPactsSP(2,:)'; % Joint 3,5
    end
    
    
    % Top Plate Joint Coords in frame of TP center:
    P_TPactsTPcen(1,1) = jointRadius*cos((pi/3)-alpha); % Joint 0 X
    P_TPactsTPcen(1,2) = jointRadius*sin((pi/3)-alpha); % Joint 0 Y
    P_TPactsTPcen(2,1) = jointRadius*cos(alpha-(pi/3)); % Joint 1 X
    P_TPactsTPcen(2,2) = jointRadius*sin(alpha-(pi/3)); % Joint 1 Y
    
    for i = 1:2
        P_TPactsTPcen(2*i+1,:) = [cosd(-120*i) -sind(-120*i) 0; sind(-120*i) cosd(-120*i) 0; 0 0 1] * P_TPactsTPcen(1,:)'; % Joint 2,4
        P_TPactsTPcen(2*i+2,:) = [cosd(-120*i) -sind(-120*i) 0; sind(-120*i) cosd(-120*i) 0; 0 0 1] * P_TPactsTPcen(2,:)'; % Joint 3,5
    end
    
    
    % Top Plate Joint Coords in SP frame:
    for i =1:6
        P_TPactsSP4(i,:) = T*[P_TPactsTPcen(i,:)';1];
        LA_lengths(i) = norm(P_TPactsSP4(i,1:3)-P_BPactsSP(i,:));
    end
    
    
    % Actuator Information:
    BaseAdd = 56.9636;
    FrontAdd = 46.45+4.4000001;
    BaseAct = 242.4;
    
    % Set the min & max stroke length for actuators
    Stroke_min = 0;
    Stroke_max = 200;
    Actuator_min = BaseAdd + FrontAdd + BaseAct + Stroke_min;
    
    
    % Calculate the step size of the acutator motor:
    step = Stroke_max/1024;
    
    
    % Calculate the actuator inputs
    Actinputs(:) = floor((LA_lengths(:) - Actuator_min)/step);
end

