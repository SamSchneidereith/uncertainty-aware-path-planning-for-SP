function [lengths] = SPConfig(TP)
%SPConfiguration: Computes Actuator inputs for given TP coordinates

jointRadius = 184.9416;
jointSpacing = 63.5;
alpha = asin((jointSpacing/2)/jointRadius);

% Set up Transformation Matrix:
P_TPoinSP = [TP(1);TP(2);TP(3)];
Rx = [ 1 0 0; 0 cos(TP(4)) -sin(TP(4)); 0 sin(TP(4)) cos(TP(4))];
Ry = [ cos(TP(5)) 0 sin(TP(5)); 0 1 0; -sin(TP(5)) 0 cos(TP(5))];
Rz = [ cos(TP(6)) -sin(TP(6)) 0; sin(TP(6)) cos(TP(6)) 0; 0 0 1];
R_TPotoSP = Rz*Ry*Rx;
T_TPotoSP = [R_TPotoSP P_TPoinSP; 0 0 0 1];

% Set up Actuator positions:
P_actsTPt = zeros(3,6); % Top joint locations on top plate
P_actsSPb = zeros(3,6); % Bot joint locations on bot plate 

P_actsSPb(1,1) = jointRadius*cos(alpha);
P_actsSPb(2,1) = jointRadius*sin(alpha);
P_actsSPb(1,2) = jointRadius*cos(alpha);
P_actsSPb(2,2) = -jointRadius*sin(alpha);

P_actsTPt(1,1) = jointRadius*cos((pi/3)-alpha);
P_actsTPt(2,1) = jointRadius*sin((pi/3)-alpha);
P_actsTPt(1,2) = jointRadius*cos((pi/3)-alpha);
P_actsTPt(2,2) = -jointRadius*sin((pi/3)-alpha);

for i = 1:2
    P_actsSPb(:,2*i+1) = [cosd(-120*i) -sind(-120*i) 0; sind(-120*i) cosd(-120*i) 0; 0 0 1]*P_actsSPb(:,1);
    P_actsSPb(:,2*i+2) = [cosd(-120*i) -sind(-120*i) 0; sind(-120*i) cosd(-120*i) 0; 0 0 1]*P_actsSPb(:,2);
    P_actsTPt(:,2*i+1) = [cosd(-120*i) -sind(-120*i) 0; sind(-120*i) cosd(-120*i) 0; 0 0 1]*P_actsTPt(:,1);
    P_actsTPt(:,2*i+2) = [cosd(-120*i) -sind(-120*i) 0; sind(-120*i) cosd(-120*i) 0; 0 0 1]*P_actsTPt(:,2);
end

% Find Top joint locations on bot plate:
[~,lengths] = ActInputsConverter(T_TPotoSP);

end

