function [Jinv] = InvJacob(SP,T)
%INVJACOB: Calculates the inverse jacobian of the SP
%   BP: State of the bot plate in {S}
%   T: Transformation Matrix of the top plate in {S}



% Set up Joint Locations:
qBPS = zeros(3,6);      %   Joints on BP in {S}
qTP4S = zeros(4,6);     %   Joints on TP in {S} with vec length 4
qTP = zeros(3,6);      %   Joints on TP in {TP} 
JinvT = zeros(6,6);
nS = zeros(3,6);

% Calculate Joint locations relative to their Plates:
ii = 0;
for i =1:3
    qBPS(:,2*ii+1) = RotMat([0,0,ii*deg2rad(-120)])*[SP.jRad*cos(SP.alpha); SP.jRad*sin(SP.alpha); 0];
    qBPS(:,2*ii+2) = RotMat([0,0,ii*deg2rad(-120)])*[SP.jRad*cos(-SP.alpha); SP.jRad*sin(-SP.alpha); 0];
    qTP(:,2*ii+1) = RotMat([0,0,ii*deg2rad(-120)])*[SP.jRad*cos((pi/3)-SP.alpha); SP.jRad*sin((pi/3)-SP.alpha); 0];
    qTP(:,2*ii+2) = RotMat([0,0,ii*deg2rad(-120)])*[SP.jRad*cos(SP.alpha-(pi/3)); SP.jRad*sin(SP.alpha-(pi/3)); 0];
    ii = ii+1;
end


for i = 1:6
    qTP4S(:,i) = T*[qTP(:,i);1];
    nS(:,i) = (qTP4S(1:3,i)-qBPS(:,i))/norm(qTP4S(1:3,i)-qBPS(:,i));
    JinvT(:,i) = [CrossP(qBPS(:,i))*nS(:,i); nS(:,i)];
end
Jinv = transpose(JinvT);
end

