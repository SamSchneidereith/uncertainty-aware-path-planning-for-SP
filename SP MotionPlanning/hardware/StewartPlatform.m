classdef StewartPlatform < handle

properties (Constant)
    height = 326.2957; % mm
end
properties
    bounds
    DoF
    W % Weight of Rotation Terms
    Q % Process Noise Covariance
    R % Observation Noise Covariance
    B, P % Joint locations on base and platform in resp. frames
    P_B  % Platform joint locations in base frame
end
methods
    function obj = StewartPlatform()
        obj.bounds.minPos = [-100, -100, obj.height];
        obj.bounds.maxPos = [ 100,  100, obj.height + 200]; 
        obj.bounds.minRot = deg2rad(-15)*[1 1 1];
        obj.bounds.maxRot = deg2rad( 15)*[1 1 1];
        obj.DoF           = 6;

        wPos              = norm(obj.bounds.maxPos-obj.bounds.minPos);
        wRot              = norm(obj.bounds.maxRot-obj.bounds.minRot);
        obj.W             = (wPos/wRot);

        Qpos = 2; Qrot = deg2rad(2); % mm, rad
        obj.Q = blkdiag(Qpos*eye(3), Qrot*eye(3));
        obj.R = LinearActuator.sigma^2 * eye(6);
        % obj.v = mvnrnd(zeros(6,1), obj.R); % MUST MAKE CONST BETWEEN TRIALS LATER
        [obj.B, obj.P] = obj.jointLocations();
        obj.P_B = zeros(3, 6);
    end

    function [l_actuators] = inverseKinematics(obj, X) % maybe take P_B out of prop and just make return
        %B: Joint positions in base Frame
        %P: Joint positions in platform Frame
        phi = X(4); theta = X(5); psi = X(6);
        Rot = StewartPlatform.eul2rot(phi, theta, psi);

        l_actuators = zeros(6, 1);
        for i = 1:6
            obj.P_B(:, i) = Rot*obj.P(:,i)+ X(1:3); 
            l_actuators(i) = norm(obj.P_B(:, i) - obj.B(:, i));
        end

        % obj.plotSP(X); % Debugging
    end

    function J = jacobian(obj, X)
        % Extract pose
        t = X(1:3);
        phi = X(4);
        theta = X(5);
        psi = X(6);
    
        % Rotation matrix
        Rot = StewartPlatform.eul2rot(phi, theta, psi);

        % Euler-rate (base angular velocity matrix)
        % cpsi = cos(psi); spsi = sin(psi);
        % ct   = cos(theta); st = sin(theta);
        % T = [cpsi*ct, -spsi, 0;
        %      spsi*ct,  cpsi, 0;
        %      -st    ,   0  , 1];
    
        % Initialize Jacobian
        J = zeros(6,6);
    
        for i = 1:6
            pB = Rot * obj.P(:,i);               % platform joint from center
            r  = t + pB - obj.B(:,i);
            l  = norm(r);
            u  = r / l;
    
            J(i,1:3) = u';                       % ∂l/∂position
            J(i,4:6) = cross(pB, u)';            % ∂l/∂Euler
    
            obj.P_B(:,i) = t + pB;               % For plotting
        end
    end

    function Jinv = inverseJacobian(obj, X)
        Jinv = pinv(obj.jacobian(X));
    end

    function [x_next, F] = DKstep(obj, x, goal, dt) % Motion Model

        % Position and orientation error
        dx = goal - x;
        
        % Desired platform twist
        Kp_pos = 1;
        Kp_rot = 0.5;
        
        v = Kp_pos * dx(1:3);
        w = Kp_rot * dx(4:6);
        
        V = [v; w];
        
        % Jacobian
        J = obj.jacobian(x);
        
        % Actuator speeds
        ldot = J * V;

        % Check actuator limits
        maxSpeed = max(abs(ldot));
        
        if maxSpeed > LinearActuator.max_vel
            scale = LinearActuator.max_vel / maxSpeed;
            V = V * scale;
        end
        
        % Recalculate Actuator speeds
        ldot = J * V;

        V_corrected = pinv(J) * ldot;
        x_next = x + V_corrected * dt;
        
        % Linearized state Jacobian (approximate)
        F = eye(6);
        
    end
    
    function [z, H] = measurementModel(obj, x)
        z = obj.inverseKinematics(x);   % actuator lengths
        H = obj.jacobian(x);            % measurement Jacobian
    end
    
    function [x1, P1] = EKFstep(obj, x0, P0, x_goal, dt)
        % Prediction
        [x_pred, F] = obj.DKstep(x0, x_goal, dt); 
        P_pred = F*P0*F' + obj.Q;
    
        % Measurement
        [z_pred, H] = measurementModel(obj, x_pred);
        z = z_pred + mvnrnd(zeros(6,1), obj.R)'; % Simulated measurements 
        y = z - z_pred; % Innovation
        % 
        % % Update
        S = H*P_pred*H' + obj.R;
        K = (P_pred*H')/S;

        x1 = x_pred + K*y;
        P1 = (eye(6) - K*H)*P_pred;
        P1 = (P1+P1')/2; % Make symmetric
    end
    
    function [x1, P1] = EKFstep_externalMeas(obj, x0, P0, x_goal, dt, z) % For Testing Purposes Only
        % Prediction
        [x_pred, F] = obj.DKstep(x0, x_goal, dt);
        P_pred = F*P0*F' + obj.Q;
    
        % Predicted measurement
        [z_pred, H] = obj.measurementModel(x_pred);
    
        % Innovation
        y = z - z_pred;
    
        % Update
        S = H*P_pred*H' + obj.R;
        K = P_pred*H'/S;
    
        x1 = x_pred + K*y;
        P1 = (eye(6) - K*H)*P_pred;
    end

    function [x, P] = propagateBelief(obj, x0, P0, x_target, dt)
        numSteps = 100;
        tol = 1;

        x = x0; P = P0;
        for k = 1:numSteps
            % [x_next, ~] = obj.DKstep(x, x_target, dt);
            % [~, P_next] = obj.EKFstep(x, P, x_next, dt);
            
            [x, P] = obj.EKFstep(x, P, x_target, dt);
            if norm(x(1:3)-x_target(1:3)) + obj.W*norm(x(4:6)-x_target(4:6)) < tol
                break;
            end
        end
    end

    function X = randState(obj)
        validState = 0; 
        while validState == 0
            % Position
            pos = obj.bounds.minPos + rand(1,3) .* (obj.bounds.maxPos - obj.bounds.minPos);
            
            % Rotation
            rot = obj.bounds.minRot + rand(1,3) .* (obj.bounds.maxRot - obj.bounds.minRot);

            X = [pos, rot]';
            l_actuators = obj.inverseKinematics(X);
            validState = LinearActuator.actCheck(l_actuators);
        end
    end

    function checkJacobian(obj,x)

        J = obj.jacobian(x);
        
        disp("Condition number:")
        disp(cond(J))
        
        s = svd(J);
        
        disp("Singular values:")
        disp(s)
        
    end

    function fig = plotSP(obj, X)
        % Extract Pose
        t = X(1:3);                 % translation (3x1)
        phi = X(4); 
        theta = X(5); 
        psi = X(6);
        
        % Rotation Matrix (ZYX convention)
        Rot = StewartPlatform.eul2rot(phi, theta, psi);
        
        % Generate Circles
        alpha = linspace(0, 2*pi, 100);
        
        % Base circle
        R_base = norm(obj.B(1:2,1));
        circle_base = [R_base*cos(alpha);
                       R_base*sin(alpha);
                       zeros(size(alpha))];
        
        % Platform circle (local frame)
        R_platform = norm(obj.P(1:2,1));
        circle_local = [R_platform*cos(alpha);
                        R_platform*sin(alpha);
                        zeros(size(alpha))];
        
        % Transform platform circle to base frame
        circle_world = Rot * circle_local + t;
        
        % Plot
        fig = figure; hold on;
        plot3(obj.B(1,:), obj.B(2,:), obj.B(3,:), ...
              'bo', 'MarkerFaceColor','b');
        plot3(circle_base(1,:), circle_base(2,:), circle_base(3,:), ...
              'b-', 'LineWidth', 1.5);
        plot3(obj.P_B(1,:), obj.P_B(2,:), obj.P_B(3,:), ...
              'ro', 'MarkerFaceColor','r');
        plot3(circle_world(1,:), circle_world(2,:), circle_world(3,:), ...
              'r-', 'LineWidth', 1.5);
        for i = 1:6
            plot3([obj.B(1,i), obj.P_B(1,i)], ...
                  [obj.B(2,i), obj.P_B(2,i)], ...
                  [obj.B(3,i), obj.P_B(3,i)], ...
                  'k-', 'LineWidth', 1);
        end
        
        axis equal
        xlim([-200,200]); ylim([-200,200]);
        grid on
        view(3)
        xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]');
        title('Stewart Platform')
    end
end

methods (Access = private)
    function [B, P] = jointLocations(~)
        % Returns base and platform joint locations in their respective frames
        B = zeros(3, 6); P = zeros(3, 6);

        jointRad = 184.9416; % Joint radius from center of plates (mm)
        jointSpacing = 63.5;    % Joint pair spacing (mm)

        alpha = asin((jointSpacing/2)/jointRad); % Half-angle between joints in joint pair (rad)
        B(:, 1) = [jointRad*cos(alpha);  jointRad*sin(alpha);  0];
        B(:, 2) = [jointRad*cos((2*pi/3) - alpha); jointRad*sin((2*pi/3) - alpha); 0];
        B(:, 3) = [jointRad*cos((2*pi/3) + alpha); jointRad*sin((2*pi/3) + alpha); 0];
        B(:, 4) = [jointRad*cos((4*pi/3) - alpha); jointRad*sin((4*pi/3) - alpha); 0];
        B(:, 5) = [jointRad*cos((4*pi/3) + alpha); jointRad*sin((4*pi/3) + alpha); 0];
        B(:, 6) = [jointRad*cos(-alpha); jointRad*sin(-alpha); 0];

        P(:, 1) = [jointRad*cos((pi/3) - alpha);  jointRad*sin((pi/3) - alpha);  0];
        P(:, 2) = [jointRad*cos((pi/3) + alpha);  jointRad*sin((pi/3) + alpha);  0];
        P(:, 3) = [jointRad*cos(pi - alpha); jointRad*sin(pi - alpha); 0];
        P(:, 4) = [jointRad*cos(pi + alpha); jointRad*sin(pi + alpha); 0];
        P(:, 5) = [jointRad*cos((5*pi/3) - alpha); jointRad*sin((5*pi/3) - alpha); 0];
        P(:, 6) = [jointRad*cos((5*pi/3) + alpha); jointRad*sin((5*pi/3) + alpha); 0];
    end
end

methods (Static)
    function Rot = eul2rot(phi, theta, psi)
        Rx = [1   0          0;
              0   cos(phi)  -sin(phi);
              0   sin(phi)   cos(phi)];

        Ry = [cos(theta)    0   sin(theta);
              0             1   0;
              -sin(theta)   0   cos(theta)];

        Rz = [cos(psi)  -sin(psi)    0;
              sin(psi)   cos(psi)    0;
              0          0           1];
        
        Rot = Rz * Ry * Rx;
    end

    function [phi, theta, psi] = rot2eul(R)
        % ZYX convention (R = Rz * Ry * Rx)
    
        theta = -asin(R(3,1));
        % Check for gimbal lock
        if abs(cos(theta)) > 1e-6
            phi = atan2(R(3,2), R(3,3));
            psi = atan2(R(2,1), R(1,1));
        else
            % Gimbal lock case
            phi = 0;
            psi = atan2(-R(1,2), R(2,2));
        end
    end
end
end

%% Notes
% -