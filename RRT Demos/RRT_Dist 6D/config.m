function cfg = config()
    % General / Experiment
    cfg.general.numTrials   = 5;
    cfg.general.maxTime     = 2*60;
    cfg.general.maxNodes    = 10*cfg.general.maxTime;
    cfg.general.stopType    = 'time';  % 'time', 'nodes' *Plotting is mostly set up for time*
    cfg.general.f_write     = 10;      % Data write frequency (Hz)

    % Algorithm
    cfg.alg.type        = 1;           % 0=RRT, 1=RRT*, 2=RRT#
    cfg.alg.goalBias    = 0.05;
    cfg.alg.delta       = 50;
    cfg.alg.ballConstant= 250;
    cfg.alg.tol         = 1;
    % costType            = 'dist';    % 'ellipsoid', 'trace', 'dist (testing)
    % rng(0.5)                         % Random Seed Fixing

    % Plotting
    cfg.plot.trial = false;
    cfg.plot.exp   = true;

    % Environment / Platform
    cfg.env.SP.height = 326.2957;
    [cfg.startNode, cfg.goalNode] = Planner.randStartGoalNodes(cfg.env.SP.height);
    cfg.env.bounds.minPos = [-100, -100, cfg.env.SP.height];
    cfg.env.bounds.maxPos = [ 100, 100, 200+cfg.env.SP.height]; 
    cfg.env.bounds.minRot = deg2rad(-15)*[1 1 1];
    cfg.env.bounds.maxRot = deg2rad( 15)*[1 1 1];
    cfg.env.dimensionMins = [cfg.env.bounds.minPos, cfg.env.bounds.minRot];
    cfg.env.dimensionMaxs = [cfg.env.bounds.maxPos, cfg.env.bounds.maxRot];
    cfg.env.wPos = norm(cfg.env.bounds.maxPos-cfg.env.bounds.minPos);
    cfg.env.wRot = norm(cfg.env.bounds.maxRot-cfg.env.bounds.minRot);
    cfg.env.W = (cfg.env.wPos/cfg.env.wRot);    % Weight on rotation terms.

    % Noise
    cfg.noise.QVal = 1; cfg.noise.RVal = 1;
    cfg.noise.pos = 2;                 % mm
    cfg.noise.ang = deg2rad(2);        % rad
    cfg.noise.QMat = cfg.noise.QVal*[cfg.noise.pos*eye(3) 1e-6*ones(3); 1e-6*ones(3) cfg.noise.ang*eye(3)];
end    


% CLEAN -------------------------------------------------------
% ---- Error Function ----
% if strcmp(costType, 'ellipsoid'), cfg.ErrFun = @VolEllipsoid2; elseif strcmp(costType, 'trace'), cfg.ErrFun = @(P) trace(P); 
%     elseif strcmp(costType, 'dist'), cfg.ErrFun = @(X) sqrt(X*X'); end
