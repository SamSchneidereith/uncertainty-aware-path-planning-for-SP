classdef Planner < handle

properties
    G, kdTree, Q
    Data
    alg
    distFun, 
    c   % Cost Function
    h   % Heuristic Condition
    epsilon
    posCheck, steer, stopingCriteriaMet
    
    delta, ballConstant, goalBias, tol
    W, % QMat, RVal
end

methods
    function obj = Planner(cfg)
        numDims = 6;
        obj.G                = motionGraph(numDims, cfg.env.dimensionMins, cfg.env.dimensionMaxs, cfg.general.maxNodes);
        obj.Q                = sbbstreeQ(cfg.general.maxNodes/10);  % Self balancing binary search tree (heap),
                                                            % this will grow if necessary
                                                            % NOTE: that because this implementation has
                                                            % only a singleton key and not a tupal key,
                                                            % the resulting algorithm may not work well
                                                            % in euclidian spaces on grids (or other
                                                            % places where cost value collisions are
                                                            % likely), it should be fine for the 2pbvp
                                                            % case here.
        obj.alg              = cfg.alg.type;
        obj.distFun          = @(A, B) norm(A(1:3)-B(1:3))+cfg.env.W*norm(A(4:6)-B(4:6));
        obj.c                = @(A, B) norm(A.position(1:3)-B.position(1:3))+cfg.env.W*norm(A.position(4:6)-B.position(4:6));
        obj.h                = @(A, B) obj.distFun(A.position, B.position);  % fallback: your weighted distance
        obj.epsilon          = cfg.alg.epsilon;   % or hard-code obj.epsilon = 10; and add decay later
        obj.kdTree           = KDTree(numDims, obj.distFun);
        % obj.QMat             = cfg.noise.QMat;
        % obj.RVal             = cfg.noise.RVal;
        obj.W                = cfg.env.W;
        obj.tol              = cfg.alg.tol;
        obj.delta            = cfg.alg.delta;
        obj.ballConstant     = cfg.alg.ballConstant;
        obj.goalBias         = cfg.alg.goalBias;
        obj.stopingCriteriaMet = cfg.stopingCriteriaMet;
        obj.posCheck         = @PosCheck;
        obj.steer            = @SPsteerDIST;

        obj.initData(cfg);
        

        startNode = ssspGraphNode(cfg.startPosition);
        goalNode = ssspGraphNode(cfg.goalPosition);
        obj.G.populateGraphNodes(startNode, goalNode);
        obj.G.startNode.g = 0; obj.G.startNode.lmc = 0;
        obj.G.goalNode.g = Inf; obj.G.goalNode.lmc = inf;
        obj.kdTree.kdInsertAsPayload(obj.G.startNode);
        fprintf('Min cost to goal: %3.f \n', obj.c(startNode, goalNode))
    end

    function ret = run(obj)
        t_start = tic;
        while ~obj.stopingCriteriaMet(toc(t_start))
            if     obj.alg == 0
                obj.RRT();
            elseif obj.alg == 1
                obj.RRTstar();
            elseif obj.alg == 2
                obj.RRTsharp();
            else
                warning('Unrecognized Algorithm Type')
                break;
            end

            % Record Data
            obj.G.extractMotionPath();
            obj.writeData(toc(t_start), obj.G.n, obj.G.goalNode.lmc, obj.G.motionPath)
        end
        obj.Data.Tree = obj.G; % Save Full Tree
        ret = obj.Data; % Return
    end

    function RRT(obj)
        x_rand = obj.G.randomPoint();
        while ~obj.posCheck(x_rand) == 0; x_rand = randomPoint(obj.G); end
        [x_near, x_near_dist] = kdFindNearestPayload(obj.kdTree, x_rand);
        traj = obj.steer(x_near.position, x_rand, obj.delta, obj.distFun);
        x_new = ssspGraphNode(x_near.position + traj);
        x_new.parentNode = x_near; x_new.lmc = x_new.parentNode.g + obj.c(x_new, x_new.parentNode);
        obj.G.insertNode(x_new);
        obj.kdTree.kdInsertAsPayload(x_new);
        x_new.g = x_new.lmc;
    end

    function RRTstar(obj)
        x_rand = obj.G.randomPoint(obj.goalBias); % Add EKF steer 
        while ~obj.posCheck(x_rand) == 0; x_rand = randomPoint(obj.G); end
        x_new = obj.extend(x_rand);
        x_new.g = x_new.lmc;
    end
    function RRTsharp(obj)
        x_rand = obj.G.randomPoint(obj.goalBias); % Add EKF steer 
        x_rand = [x_rand(1:3), 0, 0, 0];
        while ~obj.posCheck(x_rand) == 0 || ...
                obj.distFun(x_rand, obj.G.startNode.position) + obj.distFun(x_rand, obj.G.goalNode.position) >= obj.G.goalNode.g % Rejection Sampling
            x_rand = obj.G.randomPoint(obj.goalBias); 
            x_rand = [x_rand(1:3), 0, 0, 0];
        end
        obj.extend(x_rand);
        obj.replan();
    end

    function x_new = extend(obj, x_rand)
        [x_near, x_near_dist] = kdFindNearestPayload(obj.kdTree, x_rand);
        if x_near_dist == 0; return; end

        traj = obj.steer(x_near.position, x_rand, obj.delta, obj.distFun);
        x_new = ssspGraphNode(x_near.position + traj);
        x_new.parentNode = x_near; x_new.lmc = x_new.parentNode.g + obj.c(x_new, x_new.parentNode); % Move to Constructor
    
        range = hyperBallRad(obj.G, obj.delta, obj.ballConstant);
        neighbors = kdFindWithinRange(obj.kdTree, range, x_new.position);
        % neighbors = obj.kdTree.kdFindKNearestPayload(150, x_new.position);


        for n_i = 1:length(neighbors)
            neighbor = neighbors{n_i}.payload;
            % disp(length(neighbors))
            % neighbor = neighbors{n_i};

            cost = neighbor.g + obj.c(neighbor, x_new); % Will need to replace with error metrics (difficult)
            if x_new.lmc > cost 
                % fprintf("Node: %.1f, Parent: %.1f \n", obj.G.n + 1, x_new.parentNode.id)
                    % the above check is a heuristic check, if we are here than
                    % the neighbor node MAY be better, but now we need to do the
                    % harder actual 2pbvp check to make sure !!!
    
                % [neighborLocalTrajectory,neighborPMat] = steer(neighborNode.position', newPoint', delta,W,Tol,neighborNode.PMat, QMat, RVal);
                % if isempty(neighborLocalTrajectory)
                %     % warning('could not connect from neighbor')
                %     continue
                % end
                % if isnan(neighborPMat)
                %     continue
                % elseif isnan(neighborLocalTrajectory)
                %     continue
                % end
                % thisTrav = trajectoryLength(neighborLocalTrajectory);
                % thisErr = ErrFun(neighborPMat);
                % tolCheck = norm(neighborLocalTrajectory(end,:)-newPoint);
                % 
                % if thisErr+neighborNode.err < errFromStartNewPoint && tolCheck <= Tol
                %     % we have found a better parent
                % 
                %     localErr = thisErr;
                %     errFromStartNewPoint = neighborNode.err + thisErr;
                %     newPMat = neighborPMat;
                %     localTrajectory = neighborLocalTrajectory;
                %     % localTrav = thisTrav + neighborNode.travFromStart;
                %     parentNode = neighborNode;
                % end
                x_new.lmc = cost;
                x_new.parentNode = neighbor;


            end
            x_new.successors{end + 1, 1} = neighbor;
            neighbor.successors{end + 1, 1} = x_new;
        end
    
        obj.G.insertNode(x_new);
        obj.kdTree.kdInsertAsPayload(x_new);
        obj.updateQueue(x_new);

        % Goal Detection
        if obj.c(x_new, obj.G.goalNode) < obj.tol && ...
                x_new.lmc + obj.c(x_new, obj.G.goalNode) < obj.G.goalNode.lmc

            obj.G.goalNode.lmc = x_new.lmc + obj.c(x_new, obj.G.goalNode);
            obj.G.goalNode.parentNode = x_new;
            x_new.successors{end + 1, 1} = obj.G.goalNode;
            obj.G.goalNode.successors{end + 1, 1} = x_new;
            obj.updateQueue(obj.G.goalNode);
            fprintf('New Solution Found: %3.f \n', obj.G.goalNode.lmc)
        end
    end

    function replan(obj)
        while ~obj.Q.isEmpty() && obj.Q.topKey() <= min(obj.G.goalNode.g, obj.G.goalNode.lmc)
            x = obj.Q.pop();
            x.g = x.lmc;

            for n_i = 1:length(x.successors)
                successor = x.successors{n_i};
                cost = x.g + obj.c(x, successor);

                if successor == obj.G.goalNode
                    fprintf('Successor is Goal \n');
                end
    
                if successor.lmc > cost
                    % disp('Rewire');
                    if successor == obj.G.goalNode
                        fprintf('New Solution Found: %.3f (Replan)\n', cost);
                    end
                    successor.lmc = cost;
                    successor.parentNode = x;
                    obj.updateQueue(successor);


                end
            end
        end
    end
    function updateQueue(obj, x)
        key = min(x.g, x.lmc) + obj.epsilon*obj.h(x, obj.G.goalNode); % Heuristic Condition
        if x.g ~= x.lmc && x.inHeap
            obj.Q.update(x, key);
        elseif x.g ~= x.lmc && ~x.inHeap
            obj.Q.push(x, key)
        else
            obj.Q.remove(x);
        end
    end
    end

    methods(Access = private)
    function initData(obj, cfg)
        maxTime = cfg.general.maxTime;
        obj.Data = struct();
        obj.Data.Time = NaN(maxTime, 1);
        obj.Data.NumNodes = NaN(maxTime, 1);
        obj.Data.Cost = NaN(maxTime, 1);
        obj.Data.Path = cell(maxTime, 1);
        obj.Data.Tree = cell(1);
    end

    function writeData(obj, t_elapsed, numNodes, currentCost, currentPath)
        t_sec = floor(t_elapsed);
        
        if t_sec < 1 || t_sec > length(obj.Data.Time)
            return;
        end
        
        % Only write if we haven't already logged this second (avoid overwrites)
        if isnan(obj.Data.Time(t_sec))
            obj.Data.Time(t_sec)     = t_elapsed;   % actual time (not floored)
            obj.Data.NumNodes(t_sec) = numNodes;
            obj.Data.Cost(t_sec)     = currentCost;
            obj.Data.Path{t_sec}     = currentPath;  % copy of path indices or nodes
        end
    end
    end
end

%% Notes
% 
