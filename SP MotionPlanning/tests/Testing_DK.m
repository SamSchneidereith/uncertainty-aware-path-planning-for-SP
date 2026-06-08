SP = StewartPlatform();

% Initial pose
x = [0; 0; SP.height; 0; 0; 0];

% Goal pose
% goal = [50; -40; sp.height + 80; deg2rad(5); deg2rad(-4); deg2rad(6)];
goal = [5; -4; SP.height + 10; deg2rad(5); deg2rad(-4); deg2rad(6)];


dt = 1;

N = 200;

traj = zeros(6,N);

for k = 1:N
    
    traj(:,k) = x;
    
    [x, ~] = SP.DKstep(x, goal, dt);
    
    % stop if close enough
    if norm(goal - x) < 1e-3
        traj = traj(:,1:k);
        break
    end
end

% ----- Plot translation -----
figure
plot3(traj(1,:),traj(2,:),traj(3,:),'LineWidth',2)
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')
title('DKstep Cartesian Path')

% ----- Plot state convergence -----
figure
plot(traj')
legend('x','y','z','phi','theta','psi')
title('State Evolution')

% ----- Final error -----
disp("Final error:")
disp(goal - x)