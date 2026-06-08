clear, close all
SP = StewartPlatform();

x0     = [0; 0; SP.height; 0; 0; 0];
P0     = 1e-6 * eye(6);  % Small initial uncertainty, not identity
x_goal  = [5;  -4; SP.height+10;  deg2rad(5);   deg2rad(-4); deg2rad(6)];
x_goal2 = [10; -4; SP.height+10;  deg2rad(10);  deg2rad(-4); deg2rad(-5)];
x_goal3 = [11; -4; SP.height+19;  deg2rad(11);  deg2rad(-4); deg2rad(-3)];

dt = 1; numSteps = 100; tol = 1;

[x,  P,  traj,  P_trace]  = propagateBeliefTest(SP, x0, P0, x_goal,  dt, numSteps, tol);
[x2, P2, traj2, P_trace2] = propagateBeliefTest(SP, x,  P,  x_goal2, dt, numSteps, tol);
[x3, P3, traj3, P_trace3] = propagateBeliefTest(SP, x2, P2, x_goal3, dt, numSteps, tol);

% Diagnostics
fprintf('Final error Path 1:  pos=%.4f  rot=%.4f deg\n', ...
    norm(x(1:3)-x_goal(1:3)), rad2deg(norm(x(4:6)-x_goal(4:6))));
fprintf('Final error Path 2:  pos=%.4f  rot=%.4f deg\n', ...
    norm(x2(1:3)-x_goal2(1:3)), rad2deg(norm(x2(4:6)-x_goal2(4:6))));
fprintf('Final error Path 3:  pos=%.4f  rot=%.4f deg\n', ...
    norm(x3(1:3)-x_goal3(1:3)), rad2deg(norm(x3(4:6)-x_goal3(4:6))));
fprintf('Trace(P): %.4f  %.4f  %.4f\n', trace(P), trace(P2), trace(P3));

plotBelief(traj,  P_trace,  x_goal,  'Path 1');
plotBelief(traj2, P_trace2, x_goal2, 'Path 2');
plotBelief(traj3, P_trace3, x_goal3, 'Path 3');

% -------------------------------------------------------------------------
function [x, P, traj, P_trace] = propagateBeliefTest(SP, x0, P0, x_goal, dt, numSteps, tol)
    x = x0; P = P0;
    traj    = zeros(6, numSteps+1);  % Preallocate
    P_trace = zeros(1, numSteps+1);
    traj(:,1)   = x0;
    P_trace(1)  = trace(P0);

    for k = 1:numSteps
        [x, P] = SP.EKFstep(x, P, x_goal, dt);  % Approach B
        traj(:, k+1)  = x;
        P_trace(k+1)  = trace(P);
        if norm(x(1:3)-x_goal(1:3)) + SP.W*norm(x(4:6)-x_goal(4:6)) < tol
            traj    = traj(:, 1:k+1);     % Trim unused
            P_trace = P_trace(1:k+1);
            disp("break")
            break;
        end
    end
end

% -------------------------------------------------------------------------
function plotBelief(traj, P_trace, x_goal, name)
    figure('Name', name);

    subplot(3,1,1)  % 3D path
    plot3(traj(1,:), traj(2,:), traj(3,:), 'b-o', 'LineWidth', 1.5)
    hold on
    plot3(x_goal(1), x_goal(2), x_goal(3), 'r*', 'MarkerSize', 10)
    grid on; axis equal
    xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]')
    title(['Cartesian Path — ' name])
    legend('Trajectory', 'Goal')

    subplot(3,1,2)  % Orientation
    plot(rad2deg(traj(4:6,:)'))
    legend('\phi', '\theta', '\psi')
    xlabel('Step'); ylabel('Angle [deg]')
    title('Orientation Evolution')
    grid on

    subplot(3,1,3)  % Uncertainty
    plot(P_trace, 'k-', 'LineWidth', 1.5)
    xlabel('Step'); ylabel('trace(P)')
    title('Belief Uncertainty')
    grid on
end