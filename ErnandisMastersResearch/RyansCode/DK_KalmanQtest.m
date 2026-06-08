function [state,J,ldot] = DK_KalmanQtest(xstart,xgoal,dt,SP) 
%DK_Kalman returns the next state towards the goal position
% xstart: current position
% xgoal: goal position
% dt: time step
dtgen = 1;
% Initialize 
T = [RotMat(xstart(4:6)) xstart(1:3); 0 0 0 1];    % Starting Transformation Matrix
T2 = [RotMat(xgoal(4:6)) xgoal(1:3); 0 0 0 1];      % Goal Transformation Matrix
Pdiff = T2(1:3,4)-T(1:3,4);                        % Difference in position

% Set initial delta value:
del = dt*(1/8); % Fraction of distance along the path to the goal the next step should be


% Calculating Angle-axis rotation information:
w0 = RodVec(T2(1:3,1:3)*transpose(T(1:3,1:3)));
we = Normalize(w0);                      % Axis of rotation
th = norm(w0);                           % Total amount of rotation

% Calculate Initial Actuator Lengths:
LAcalc = ActInputsConverter(T);         % Calculated actuator lengths            
% DK "Loop":
pdel = del*Pdiff;                       % Fraction of distance to move
pDot = (pdel/dtgen);                    % Velocity to move
th_del = del*th;                        % Fraction of angle to rotate
phiDot = (th_del/dtgen);                % Angular velocity to move 
wS = we*phiDot;                         % Axis-Angle velocity
RDel = expm(CrossP(we)*th_del);         % Rotation matrix for angle change
VS = [wS; pDot-CrossP(wS)*T(1:3,4)];    % Velocity vector (Angular; Linear)
Jinv = InvJacob(SP,T);                  % Inverse jacobian
ldot = Jinv*VS;                         % Actuator speed to move to next position
% ldotorig= ldot;                         % Keep for comparison if next while loop triggered      
while max(abs(ldot)) > 3.5              % Check to see if LA speed is higher than max, if so adjust
    del = del/2;                        % Lower fraction distance and do above again
    pdel = del*Pdiff;
    pDot = (pdel/dtgen);
    th_del = del*th;
    phiDot = (th_del/dtgen);
    wS = we*phiDot;
    RDel = expm(CrossP(we)*th_del);
    VS = [wS; pDot-CrossP(wS)*T(1:3,4)];
    Jinv = InvJacob(SP,T);
    ldot = Jinv*VS;                             % Adjusted actuator speed
end
J = inv(Jinv);                                  % Non-inverted Jacobian
T = [RDel*T(1:3,1:3) pdel+T(1:3,4); 0 0 0 1];   % Update Position
LAcalc = (ldot*dtgen) + LAcalc;                 % Calculated actuator lengths
LAact = ActInputsConverter(T);                  % Actual (IK) actuator lengths
LAdiff = LAact - LAcalc;                        % Difference in actuator lengths
state = [T(1:3,4); Mat2Euler(T(1:3,1:3))];      % Updated state

end