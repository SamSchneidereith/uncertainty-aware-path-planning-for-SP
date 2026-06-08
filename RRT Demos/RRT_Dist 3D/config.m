function cfg = config()
    % General / Experiment
    cfg.general.numTrials   = 1;
    cfg.general.maxTime     = 0.2*60;
    cfg.general.maxNodes    = 10*cfg.general.maxTime;
    endCriteria             = 'time';   % 'time', 'nodes', 'cost' *Plotting is mostly set up for time*

    % Algorithm
    cfg.alg.type        = 2;        % 0=RRT, 1=RRT*, 2=RRT#
    cfg.alg.goalBias    = 0.05;
    cfg.alg.delta       = 20;
    cfg.alg.ballConstant= 100;
    cfg.alg.tol         = 1;
    cfg.alg.epsilon     = 1;
    costType            = 'dist';   % 'ellipsoid', 'trace', 'dist (testing)
    % rng(0.5)                      % Random Seed Fixing

    % Plotting
    cfg.plot.trial = true;
    cfg.plot.exp   = true;

    % Environment / Platform
    [cfg.startPosition, cfg.goalPosition] = randStartGoalPos();
    cfg.startPosition = [cfg.startPosition(1:3), 0, 0, 0]; cfg.goalPosition = [cfg.goalPosition(1:3), 0, 0, 0];
    cfg.env.SP.height = 326.2957;
    cfg.env.bounds.minPos = [-100, -100, cfg.env.SP.height];
    cfg.env.bounds.maxPos = [ 100, 100, 200+cfg.env.SP.height]; 
    cfg.env.bounds.minRot = deg2rad(-15)*[1 1 1];
    cfg.env.bounds.maxRot = deg2rad( 15)*[1 1 1];
    cfg.env.dimensionMins = [cfg.env.bounds.minPos, cfg.env.bounds.minRot];
    cfg.env.dimensionMaxs = [cfg.env.bounds.maxPos, cfg.env.bounds.maxRot];
    cfg.env.wPos = norm(cfg.env.bounds.maxPos-cfg.env.bounds.minPos);
    cfg.env.wRot = norm(cfg.env.bounds.maxRot-cfg.env.bounds.minRot);
    cfg.env.W = (cfg.env.wPos/cfg.env.wRot);    % Weight on rotation terms.
    cfg.env.W = 0; % Debug

    % Noise
    cfg.noise.QVal = 1; cfg.noise.RVal = 1;
    cfg.noise.pos = 2;                 % mm
    cfg.noise.ang = deg2rad(2);        % rad
    cfg.noise.QMat = cfg.noise.QVal*[cfg.noise.pos*eye(3) 1e-6*ones(3); 1e-6*ones(3) cfg.noise.ang*eye(3)];
    


    % CLEAN BELOW ME -------------------------------------------------------

    % ---- Error Function ----
    if strcmp(costType, 'ellipsoid'), cfg.ErrFun = @VolEllipsoid2; elseif strcmp(costType, 'trace'), cfg.ErrFun = @(P) trace(P); 
        elseif strcmp(costType, 'dist'), cfg.ErrFun = @(X) sqrt(X*X'); end

    % ---- Stoppoing Criteria ----
    if endCriteria == 'time'
        cfg.stopingCriteriaMet =@(tcur) tcur >= cfg.general.maxTime;
    elseif endCriteria == 'nodes'
        cfg.stopingCriteriaMet = @(graph)(graph.n >= graph.maxNodesAllowed);     
    elseif endCriteria == 'error'
        cfg.stopingCriteriaMet = @(graph) (~isinf(norm(graph.goalNode.err)) || graph.n >= graph.maxNodesAllowed);
    else
        error('unknown stopping criteria function')
    end

    % ---- Matlab Settings ----
    % set(0,'DefaultFigureVisible','off'); % Hides Plots
    % dbstop if error
    % warning('off', 'MATLAB:bvp4c:RelTolNotMet');   
    % turn off this particular warning about convergence tolerances not being met (we just throw out points of which this happens)
        % these are the start/goal positions (note that they are reset
        % to the closest nodes in the graph if the graph is loaded
        % from a file), if instead we want to use a node ID, then
        % that can be done with the following option
    end

function [startPosition, goalPosition] = randStartGoalPos()
    check = 1; SP.height = 326.2957; 
    while check == 1 % Rand Start Pos  
        startPosition = [-100+200*rand(), -100+200*rand(), SP.height+200*rand(), deg2rad(-15+30*rand()), deg2rad(-15+30*rand()), deg2rad(-15+30*rand())];
        check = PosCheck(startPosition);   
    end

    check = 1;
    while check == 1 % Rand Goal Pos  
        goalPosition = [-100+200*rand(), -100+200*rand(), SP.height+200*rand(), deg2rad(-15+30*rand()), deg2rad(-15+30*rand()), deg2rad(-15+30*rand())];
        check = PosCheck(goalPosition);
    end
     
    % Extremes:
    %  % Bottom to Top:
    % StartSet(6,:) = [-100, -100, SP.height+5, deg2rad(5), deg2rad(5), deg2rad(5)];
    % GoalSet(6,:) = [100, 100, SP.se height+160, deg2rad(-5), deg2rad(-5), deg2rad(-5)];
    % 
    %  %Top to Bottom:
    % StartSet(7,:) = [-100, 100, SP.height+160, deg2rad(5), deg2rad(5), deg2rad(5)];
    % GoalSet(7,:) = [100, -100, SP.height+5, deg2rad(-10), deg2rad(5), deg2rad(-10)];
    % 
    %  % Vertical Motion with Large Rotation:
    % StartSet(8,:) = [0, 0, SP.height+50, deg2rad(-15), deg2rad(-15), deg2rad(-15)];
    % GoalSet(8,:) = [0, 0, SP.height+200, 0, 0, 0];
    % 
    %  % Large Translation on same height:
    % StartSet(9,:) = [100, -100, SP.height+100, deg2rad(15), deg2rad(-15), deg2rad(15)];
    % GoalSet(9,:) = [-100, 100, SP.height+100, deg2rad(-15), deg2rad(15), deg2rad(-15)];
    % 
    %  % Pure straight Line Translation:
    % StartSet(10,:) = [0, -100, SP.height+100, deg2rad(0), deg2rad(0), deg2rad(0)];
    % GoalSet(10,:) = [0, 100, SP.height+100, deg2rad(0), deg2rad(0), deg2rad(0)]; 
end