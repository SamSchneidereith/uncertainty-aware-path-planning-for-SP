% EKF Testing
% Truth State
addpath('tests')
addpath('hardware')
SP = StewartPlatform();
% SP.R = 10.*SP.R; % test

dt = 0.05;
goal = [50 40 SP.height deg2rad(5) 0 0]';

% True state
x_true = [0 0 SP.height 0 0 0]';

% Estimated state (wrong on purpose)
x_est = x_true + [20 -15 0 deg2rad(4) 0 0]';

P = eye(6)*50;

steps = 200;

err = zeros(6,steps);
traceP = zeros(1,steps);

% Truth Simulation
for k = 1:steps

    % ---- TRUE MOTION ----
    % [x_true, ~] = SP.motionModel(x_true, goal, dt);
    Kp_true = 1;   % slightly different than EKF's Kp = 1
    x_true = x_true + Kp_true*(goal - x_true)*dt;

    % ---- TRUE MEASUREMENT ----
    z_true = SP.inverseKinematics(x_true);
    z_meas = z_true + mvnrnd(zeros(6,1), SP.R)';

    % ---- EKF STEP ----
    [x_est, P] = EKFstep_externalMeas(SP, x_est, P, goal, dt, z_meas);

    err(:,k) = x_true - x_est;
    traceP(k) = trace(P);

end

figure
subplot(3,1,1)
plot(vecnorm(err(1:3,:)))
title('Position Error Norm')
ylabel('Error (mm)')

subplot(3,1,2)
plot(vecnorm(err(4:6,:)))
title('Rotation Error Norm')
ylabel('Error (rad)')

subplot(3,1,3)
plot(traceP)
title('Trace of Error Covariance')
xlabel('Time (s)')