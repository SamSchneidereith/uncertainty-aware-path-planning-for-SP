function [x,P] = SPSteerV2(Start,Goal,delta,W,Tol,P,Q)
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
R = [0.308764757748884	-0.00222047549336229	-0.00474512475051407	-5.92116799337972e-06	0.000190262896200763	-2.45290047621554e-05;
-0.00222047549336229	0.304223091029725	0.00231259496481015	-0.000193734589072858	-8.39059440479655e-06	9.54266614634882e-06;
-0.00474512475051407	0.00231259496481015	0.0273266552929653	-6.96961649889598e-07	-5.21077733083947e-06	3.69373446441588e-06;
-5.92116799337972e-06	-0.000193734589072858	-6.96961649889598e-07	1.18983311549153e-06	-2.39755444701845e-09	6.17041275323271e-08;
0.000190262896200763	-8.39059440479655e-06	-5.21077733083947e-06	-2.39755444701845e-09	1.16842335807103e-06	-1.76732841594943e-08;
-2.45290047621554e-05	9.54266614634882e-06	3.69373446441588e-06	6.17041275323271e-08	-1.76732841594943e-08	4.65849636711235e-06];

% Q =0.2904*eye(6);  % Process Noise Covariance
%{
Poserr = 2;          % Assuming an initial position error of 2 mm
Angerr = deg2rad(2); % Assuming an initial angle error of 1 degrees
Q = [Poserr*eye(3) zeros(3); zeros(3) Angerr*eye(3)];
%}
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
while k < N && distGoal > Tol
    k = k+1;
%for k = 2:N
    % Get True state:
    [xtrue(:,k),~] = DK_Kalman(xtrue(:,k-1),Goal,dt,SP);
    % EKF Prediction Step:
    % A priori state estimate: x = f(x-1,u)
    [x(:,k),J] = DK_Kalman(x(:,k-1),Goal,dt,SP);
    % A priori error covariance: P = JPJ'+Q;
    P = J*P*J' + Q;
    P = (P+P')/2;
    % Make Observation: % z=h(x) + v
    v = 5*normrnd(0,diag(R));
    z(:,k) = xtrue(:,k) + v;

    % Update Step:
    % Innovation/Pre-fit residual: y = z-h(x)
    y = z(:,k) - xtrue(:,k);
    % Innovation covariance: S=HPH'+R
    [H,Hall] = HJ(x(:,k),Hall,h);

    S = H*P*H'+R;
    S = (S+S')/2;
    % Fix rounding errors in S:
    
    % Near-Optimal Kalman gain: K=PH'S^-1
    K = P*H'/S;
    % Update state estimate: x = x+Ky
    x(:,k) = x(:,k) +K*y;
    % Update error covariance: P=(I-KH)P
    P =(eye(6)-K*H)*P;
    P = (P+P')/2;
    distGoal = norm(x(1:3,k)-Goal(1:3))+W*norm(x(4:6,k)-Goal(4:6));
end
x = x';
end

