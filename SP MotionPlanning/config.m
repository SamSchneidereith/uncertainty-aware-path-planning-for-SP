function cfg = config(SP)
    % General / Experiment
    cfg.general.numTrials   = 3;
    cfg.general.maxTime     = 1*60;    % seconds
    cfg.general.maxNodes    = 10*cfg.general.maxTime;
    cfg.general.stopType    = 'time';  % 'time', 'nodes' *Plotting is mostly set up for time*
    cfg.general.f_write     = 10;      % Data write frequency (Hz)

    % Algorithm
    cfg.alg.type        = 2;           % 0=RRT, 1=RRT*, 2=RRT#
    cfg.alg.goalBias    = 0.05;
    cfg.alg.delta       = 125;
    cfg.alg.ballConstant= 250;
    cfg.alg.tol         = 1;
    % rng(0.5)                         % Random Seed Fixing [Disabled]

    % Plotting
    cfg.plot.trial = false;
    cfg.plot.exp   = true;

    cfg.startNode = ssspGraphNode(SP.randState());
    cfg.goalNode  = ssspGraphNode(SP.randState());
    cfg.P_start = 1e-3 * eye(SP.DoF);

    addpath('planning')
    addpath('dataStructures')
    addpath('hardware')
    addpath('analysis')
    addpath('tests')
end
