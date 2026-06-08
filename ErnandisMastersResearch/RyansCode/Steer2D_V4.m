function [X,Pout] = Steer2D_V4(Start,Goal,delta,W,Tol,P,Q,RVal)
% 2D Steering Function:
% Function inputs:
% Min actuator move = 200/1024;
% Min norm position move = 0.1953
% Min vertical or horizontal move = 0.2762
% Min diagonal move = 0.1381
step = 200/1024;
N = max(2,round(delta/step));

% P = eye(2).*diagP;
% P = zeros(2);
% EKF Conditions:
R = RVal*[0.0844 -2.8739e-6; -2.8739e-6 0.0843]; % Observation Noise
%Q = 5*[0.0844 -5.0781e-5; -5.0781e-5 0.0844]; % Process Noise Covariance
%P = [1 0; 0 1]; % Initial Error Covariance
H = [1 0; 0 1]; % Observation Matrix
J = [cosd(45) -sind(45); sind(45) cosd(45)]; % Jacobian
goalDist = norm(Goal-Start);
% Initialize arrays:
X(:,1) = Start;
Xtrue(:,1) = Start;
z(:,1) = Start;
k = 1;
check = 0;
while check == 0
    k = k+1;
    % EKF Function:
    % Get True State:
    [Xtrue(:,k)] = MoveAct(Xtrue(:,k-1),Goal,step);
    check2 = PosCheck2D(Xtrue(:,k));
    % Prediction State Estimate:
    [X(:,k)] = MoveAct(X(:,k-1),Goal,step);
    check3 = PosCheck2D(X(:,k));
    % Prediction Error Covariance:
    P(:,:,k) = J*P(:,:,k-1)*J' + Q;
    % Observation:
    % Add noise depending on position:
    w = NoiseAddition(Xtrue(:,k));
    v = w.*normrnd(0,diag(R));
    z(:,k) = Xtrue(:,k) + v;
    % Innovation/Measurement Prefit Residual:
    y = z(:,k) - H*Xtrue(:,k);
    % Innovation Covariance:
    S = H*P(:,:,k)*H'+R;
    % Near-Optimal Kalman Gain:
    K = P(:,:,k)*H'/S;
    % Update State Estimate:
    X(:,k) = X(:,k) + K*y;
    % Update Error Covariance:
    P(:,:,k) = (eye(2)-K*H)*P(:,:,k);
    goalDist = norm(Goal-Xtrue(:,k));
    if check2 == 1
        check = 2;
    elseif check3 == 1
        check = 2;
    elseif goalDist < Tol
        check = 1;
    elseif k == N
        check = 1;
    else 
        check = 0;
    end
end
if check == 1
    Pout = P(:,:,end);
    X = X';
else
    X = NaN(size(X,1),1);
    Pout = NaN(size(P(:,:,end)));
end
    function w = NoiseAddition(Xtrue)
        % Further up and to the right the position is, more error there is:
        w = zeros(2,1);
        Xrange = 145-(-145);
        Xmin = -145;
        Yrange = 790-490;
        Ymin = 490;
        w(1) = (Xtrue(1)-Xmin)/Xrange;
        w(2) = (Xtrue(2)-Ymin)/Yrange;        
    end
end