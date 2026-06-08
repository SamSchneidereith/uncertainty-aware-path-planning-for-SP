clear
clc
close all

disp("Creating Stewart platform...")
sp = StewartPlatform();

% Nominal pose
% x = [0;0;sp.height;0;0;0];
x = [10;5;sp.height+30;deg2rad(2);deg2rad(-1);deg2rad(1)];
cond(sp.jacobian(x))

disp(" ")
disp("Testing inverse kinematics...")

l = sp.inverseKinematics(x)

disp("Expected:")
disp("All 6 actuator lengths roughly similar")

% Jacobian test
disp(" ")
disp("Testing Jacobian via finite difference...")

J = sp.jacobian(x);

dx = [0.001;0.001;0.001;deg2rad(0.1);deg2rad(0.1);deg2rad(0.1)];

x2 = x + dx;

l1 = sp.inverseKinematics(x);
l2 = sp.inverseKinematics(x2);

dl_true = l2 - l1;
dl_est  = J*dx;

disp("True actuator change:")
disp(dl_true)

disp("Jacobian predicted change:")
disp(dl_est)

disp("Error:")
disp(dl_true - dl_est)

disp("Jacobian condition number:")
disp(cond(J))

% EKF propagation test

disp(" ")
disp("Testing EKF covariance propagation...")

P = 10*eye(6);      % large initial uncertainty
goal = x;

trace_history = zeros(100,1);

for k=1:100
    
    [x,P] = sp.EKFstep(x,P,goal, 0.1);
    
    trace_history(k) = trace(P);
    
end

figure
plot(trace_history)
xlabel("Step")
ylabel("trace(P)")
title("Covariance Trace Evolution")

disp("Expected behavior:")
disp("- trace(P) should stabilize")
disp("- It should NOT explode")
disp("- It should NOT go negative")