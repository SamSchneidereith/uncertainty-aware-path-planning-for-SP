function [x,xtrue] = SPSteer(Start,Goal,delta,W,Tol)
%SPSTEER: Drives the SP towards a goal position from a start position

% EKF Conditions:
h = 0.1;        % Finite difference
dt = 0.1;       % time steps
T = 1:dt:delta;    % total time
N = length(T);  % Number of steps

% SP Physical Aspects:
SP.jRad = 184.9416;     % distance from plate center to joint [mm]
SP.jSpace = 63.5;       % Space between joints per pair [mm]
SP.alpha = asin((SP.jSpace/2)/SP.jRad); % 1/2 angle between joint pairs
SP.height = 326.2957;   % Base height from bottom plate to top plate [mm]

% EKF Noises:
% Observation Noise:
R = [0.3925,    -1.6071e-2,  -3.6872e-2,  3.5333e-5,   1.985e-4,   -5.6667e-5;
    -1.6071e-2, 0.3735,      -3.4887e-2,  -2.12e-4,    3.9e-5,     6.4167e-5;
    -3.6872e-2,	-3.4887e-2,  2.2961e-2,   1.6167e-5,   -2.5667e-5, -1.5833e-5;
    3.5333e-5,	-2.12e-4,    1.6167e-5,   1.0e-6,      1.0e-6,     6.6667e-7;
    1.985e-4,	3.9e-5,      -2.5667e-5,  1.0e-6,      1.0e-6,     6.6667e-7;
    -5.6667e-5, 6.4167e-5,   -1.5833e-5,  6.6667e-7,   6.6667e-7,  5.8333e-6];

Q = [0.01*eye(3) zeros(3); zeros(3) 0.01*eye(3)];  % Process Noise Covariance

Poserr = 1;          % Assuming an initial position error of 2 mm
Angerr = deg2rad(1); % Assuming an initial angle error of 1 degrees
P = [ Poserr*eye(3) zeros(3); zeros(3) Angerr*eye(3)];

% Initialize Vectors:
%x = zeros(6,N);
%xtrue = zeros(6,N);
%z = zeros(6,N);
Hall = double.empty(6,6,0);  % Initialize H matrix

% Set initial position:
x(:,1) = Start;
z(:,1) = Start;
xtrue(:,1) = Start;
distGoal = norm(x(1:3,1)-Goal(1:3))+W*rad2deg(norm(x(4:6,1)-Goal(4:6)));
k = 1;
while k<N && distGoal > Tol
    k = k+1;
%for k = 2:N
    % Get True state:
    [xtrue(:,k),~] = DK_Kalman(xtrue(:,k-1),Goal,dt,SP);
    % EKF Prediction Step:
    % A priori state estimate: x = f(x-1,u)
    [x(:,k),J] = DK_Kalman(x(:,k-1),Goal,dt,SP);
    % A priori error covariance: P = JPJ'+Q;
    P = J*P*J' + Q;
    % Make Observation: % z=h(x) + v
    v = 5*normrnd(0,diag(R));
    z(:,k) = xtrue(:,k) + v;

    % Update Step:
    % Innovation/Pre-fit residual: y = z-h(x)
    y = z(:,k) - xtrue(:,k);
    % Innovation covariance: S=HPH'+R
    [H,Hall] = HJ(x(:,k),Hall,h);

    S = H*P*H'+R;
    % Near-Optimal Kalman gain: K=PH'S^-1
    K = P*H'/S;
    % Update state estimate: x = x+Ky
    x(:,k) = x(:,k) +K*y;
    % Update error covariance: P=(I-KH)P
    P =(eye(6)-K*H)*P;
    
    distGoal = norm(x(1:3,k)-Goal(1:3))+W*norm(x(4:6,k)-Goal(4:6));
end
x = x';
xtrue = xtrue';
end

