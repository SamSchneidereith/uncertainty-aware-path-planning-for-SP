classdef Planner < handle

properties
    G, kdTree, Q
    Data, f_write
    alg
    metric, c, h   % Metric, Cost, and Heuristic Functions resp.
    steer, stoppingCriteriaMet
    delta, ballConstant, goalBias, tol
    W, QMat, RVal

    t_start
end

methods
    function obj = Planner(cfg)
        numDims = 6;
        obj.G                = motionGraph(numDims, cfg.env.dimensionMins, cfg.env.dimensionMaxs, cfg.general.maxNodes);
        obj.Q                = sbbstreeQ(cfg.general.maxNodes/10);  % Self balancing binary search tree (heap)
        obj.alg              = cfg.alg.type;
        obj.W                = cfg.env.W;
        obj.metric           = @(A, B) Planner.distFun(A, B, obj.W);
        obj.c                = @(A, B) obj.metric(A.position, B.position);
        obj.h                = @(A, B) obj.c(A, B);  % heuristic condition
        obj.kdTree           = KDTree(numDims, obj.metric);
        % obj.QMat             = cfg.noise.QMat;
        % obj.RVal             = cfg.noise.RVal;
        
        obj.tol              = cfg.alg.tol;
        obj.delta            = cfg.alg.delta;
        obj.ballConstant     = cfg.alg.ballConstant;
        obj.goalBias         = cfg.alg.goalBias;
        obj.steer            = @SPsteer;

        obj.f_write          = cfg.general.f_write;
        obj.initData(cfg);
        
        switch cfg.general.stopType
            case 'time'
                obj.stoppingCriteriaMet = @(t) t >= cfg.general.maxTime;
            case 'nodes'
                obj.stoppingCriteriaMet = @(t) obj.G.n >= cfg.general.maxNodes;
        end

        startNode = cfg.startNode; goalNode = cfg.goalNode;
        obj.G.populateGraphNodes(startNode, goalNode);
        obj.kdTree.kdInsertAsPayload(obj.G.startNode);
        fprintf('Min cost to goal: %3.f \n', obj.c(startNode, goalNode))
    end

    function trialData = run(obj)
        obj.t_start = tic;
        while ~obj.stoppingCriteriaMet(toc(obj.t_start))
            if     obj.alg == 0
                obj.RRT();
            elseif obj.alg == 1
                obj.RRTstar();
            elseif obj.alg == 2
                obj.RRTsharp();
            else
                error('Unrecognized Algorithm Type')
            end

            % Record Data
            obj.G.extractMotionPath();
            obj.writeData(toc(obj.t_start), obj.G.n, hyperBallRad(obj.G, obj.delta, obj.ballConstant), obj.G.goalNode.g, obj.G.motionPath)
        end
        obj.interpolateData();
        obj.Data.alg = obj.alg;
        obj.Data.Tree = obj.G; % Save Full Tree
        trialData = obj.Data;        % Return
    end

    function RRT(obj)
        x_rand = ssspGraphNode(obj.G.randomPoint(obj.goalBias));
        while ~Planner.validState(x_rand)
            x_rand = ssspGraphNode(obj.G.randomPoint(obj.goalBias)); 
        end
        [x_near, x_near_dist] = kdFindNearestPayload(obj.kdTree, x_rand.position);
        if x_near_dist == 0; return; end % Avoids resampling existing nodes
        traj = obj.steer(x_near, x_rand, obj.delta, obj.metric);
        x_new = ssspGraphNode(x_near.position + traj, x_near);
        if ~Planner.validEdge(x_new, x_near); return; end
        x_new.g = x_new.parent.g + obj.c(x_new, x_new.parent);
    
        obj.G.insertNode(x_new);
        obj.kdTree.kdInsertAsPayload(x_new);
        obj.updateQueue(x_new);

        % Goal Detection
        if obj.c(x_new, obj.G.goalNode) < obj.tol && ...
                x_new.g + obj.c(x_new, obj.G.goalNode) < obj.G.goalNode.g && ...
                Planner.validEdge(x_new, obj.G.goalNode)

            obj.G.goalNode.g = x_new.g + obj.c(x_new, obj.G.goalNode);
            obj.G.goalNode.parent = x_new;
            fprintf('New Solution Found: %3.f \n', obj.G.goalNode.g);
        end
    end

    function RRTstar(obj)
        x_rand = ssspGraphNode(obj.G.randomPoint(obj.goalBias));
        while ~Planner.validState(x_rand) || ...
                Planner.distFun(x_rand, obj.G.startNode, obj.W) + Planner.distFun(x_rand, obj.G.goalNode, obj.W) >= obj.G.goalNode.g % Rejection Sampling
            x_rand = ssspGraphNode(obj.G.randomPoint(obj.goalBias));
                       
            if obj.stoppingCriteriaMet(toc(obj.t_start)); return; end
        end
        x_new = obj.extend(x_rand);
        if ~isempty(x_new); x_new.g = x_new.lmc; end
        
        obj.G.goalNode.g = obj.G.goalNode.lmc; % Allows for common extend fun. between RRT* and RRT#
    end
    
    function RRTsharp(obj)
        x_rand = ssspGraphNode(obj.G.randomPoint(obj.goalBias));
        while ~Planner.validState(x_rand) || ...
                Planner.distFun(x_rand, obj.G.startNode, obj.W) + Planner.distFun(x_rand, obj.G.goalNode, obj.W) >= obj.G.goalNode.g % Rejection Sampling
            x_rand = ssspGraphNode(obj.G.randomPoint(obj.goalBias));

            if obj.stoppingCriteriaMet(toc(obj.t_start)); return; end
        end
        obj.extend(x_rand);
        obj.replan();
    end

    function x_new = extend(obj, x_rand)
        [x_near, x_near_dist] = kdFindNearestPayload(obj.kdTree, x_rand.position);
        if x_near_dist == 0; x_new = []; return; end           % Avoids resampling existing nodes
        traj = obj.steer(x_near, x_rand, obj.delta, obj.metric);
        x_new = ssspGraphNode(x_near.position + traj, x_near);
        if ~Planner.validEdge(x_new, x_near); x_new = []; return; end  % Verify possible edge
        x_new.lmc = x_new.parent.g + obj.c(x_new, x_new.parent);
    
        range = hyperBallRad(obj.G, obj.delta, obj.ballConstant);
        neighbors = kdFindWithinRange(obj.kdTree, range, x_new.position);

        for n_i = 1:length(neighbors)
            neighbor = neighbors{n_i}.payload;
            if Planner.validEdge(x_new, neighbor)
                if x_new.lmc > neighbor.g + obj.c(neighbor, x_new)    
                    x_new.lmc = neighbor.g + obj.c(neighbor, x_new);
                    x_new.parent = neighbor;
                end
                x_new.successors{end + 1, 1} = neighbor;
                neighbor.successors{end + 1, 1} = x_new;
            end
        end
        
        x_new.g = x_new.lmc; % RRT* CODE
        if obj.alg == 1
            for neighbor = 1:length(neighbors)
               if neighbor.g > x_new.g + obj.c(x_new, neighbor)
                    neighbor.g      = x_new.g + obj.c(x_new, neighbor);
                    neighbor.lmc    = neighbor.g;
                    neighbor.parent = x_new;
                    obj.propagateCost(neighbor);  % <-- propagate to children
               end
            end
        end
    
        obj.G.insertNode(x_new);
        obj.kdTree.kdInsertAsPayload(x_new);
        obj.updateQueue(x_new);

        % Goal Detection
        if obj.c(x_new, obj.G.goalNode) < obj.tol && ...
                x_new.lmc + obj.c(x_new, obj.G.goalNode) < obj.G.goalNode.lmc && ...
                Planner.validEdge(x_new, obj.G.goalNode)

            obj.G.goalNode.lmc = x_new.lmc + obj.c(x_new, obj.G.goalNode);
            obj.G.goalNode.parent = x_new;
            x_new.successors{end + 1, 1} = obj.G.goalNode;
            obj.G.goalNode.successors{end + 1, 1} = x_new;
            fprintf('New Solution Found: %3.f \n', obj.G.goalNode.lmc);
            obj.updateQueue(obj.G.goalNode);
        end
    end

    function propagateCost(obj, x) % RRTstar
        for n_i = 1:length(x.successors)
            successor = x.successors{n_i};
            new_cost = x.g + obj.c(x, successor);
            if successor.g > new_cost
                successor.g   = new_cost;
                successor.lmc = new_cost;
                successor.parent = x;
                obj.propagateCost(successor);  % recurse down the subtree
            end
        end
    end

    function replan(obj)
        while ~obj.Q.isEmpty() && topKey(obj.Q) <= min(obj.G.goalNode.g, obj.G.goalNode.lmc)
            x = obj.Q.pop();
            x.g = x.lmc;

            for n_i = 1:length(x.successors)
                successor = x.successors{n_i};
                cost = x.g + obj.c(x, successor);
    
                if successor.lmc > cost
                    successor.lmc = cost;
                    successor.parent = x;
                    obj.updateQueue(successor);

                    if successor == obj.G.goalNode
                        fprintf('New Solution Found: %.3f (Replan)\n', cost);
                    end
                end
            end
        end
    end

    function updateQueue(obj, x)
        key = min(x.g, x.lmc) + obj.h(x, obj.G.goalNode); % Heuristic Condition
        if x.g ~= x.lmc && x.inHeap
            obj.Q.update(x, key);
        elseif x.g ~= x.lmc && ~x.inHeap
            obj.Q.push(x, key)
        elseif x.g == x.lmc && x.inHeap
            obj.Q.remove(x);
        end
    end
end

methods(Access = private)
    function initData(obj, cfg)
        maxTime = cfg.general.maxTime;
        numCol = maxTime*obj.f_write;
        obj.Data = struct();
        obj.Data.alg = NaN;
        obj.Data.Time = NaN(numCol, 1);
        obj.Data.NumNodes = NaN(numCol, 1);
        obj.Data.BallRad = NaN(numCol,1);
        obj.Data.Cost = NaN(numCol, 1);
        obj.Data.Path = cell(numCol, 1);
        obj.Data.Tree = cell(1);
    end

    function writeData(obj, t_elapsed, numNodes, ballRad, currentCost, currentPath)
        if t_elapsed < 1/obj.f_write || t_elapsed*obj.f_write > length(obj.Data.Time)
            return;
        end
        
        dt = 1 / obj.f_write;
        slot_idx = floor(t_elapsed / dt) + 1;  % 1-based indexing
        
        % Safety: don't write beyond pre-allocated size
        if slot_idx > length(obj.Data.Time)
            warning('writeData: time exceeded pre-allocated slots (%d > %d)', ...
                    slot_idx, length(obj.Data.Time));
            return;
        end

        if isnan(obj.Data.Time(slot_idx)) % Don't overwrite
            obj.Data.Time(slot_idx)     = t_elapsed;       % actual time
            obj.Data.NumNodes(slot_idx) = numNodes;
            obj.Data.BallRad(slot_idx)  = ballRad;
            obj.Data.Cost(slot_idx)     = currentCost;
            obj.Data.Path{slot_idx}     = currentPath;
        end
    end

    function interpolateData(obj)
        % Any cell that is NaN due to skipped writing copies previous cell
        for i = 2:length(obj.Data.Time)
            if isnan(obj.Data.Time(i));     obj.Data.Time(i) =     obj.Data.Time(i - 1);    end
            if isnan(obj.Data.NumNodes(i)); obj.Data.NumNodes(i) = obj.Data.NumNodes(i - 1);end
            if isnan(obj.Data.BallRad(i));  obj.Data.BallRad(i) =  obj.Data.BallRad(i - 1); end
            if isnan(obj.Data.Cost(i));     obj.Data.Cost(i) =     obj.Data.Cost(i - 1);    end
            if isnan(obj.Data.Path{i});     obj.Data.Path{i} =     obj.Data.Path{i - 1};    end
        end
    end
end

methods(Static)
    function [dist] = distFun(A, B, W)
        % Calculates Euclidean distance between two states
        % Accepts both ssspGraphNode and numeric position vector
    
        if isprop(A, 'position'); A = A.position; end
        if isprop(B, 'position'); B = B.position; end

        dist = norm(A(1:3)-B(1:3)) + W*norm(A(4:6)-B(4:6));
    end

    function bool = validState(x)
        % Accepts both ssspGraphNode and numeric position vector
    
        if isnumeric(x)
            pos = x;
        elseif isprop(x, 'position')
            pos = x.position;
        else
            error('validState: input must be a node or numeric position vector');
        end
    
        LA = SPConfig(pos);
        bool = ActCheck(LA);
    end

    function [bool] = validEdge(A, B)
        collisionResolution = 10; % mm
        if A.position == B.position; bool = true; return; end % If coinciding nodes, break, return true

        direction = B.position - A.position;
        dist = norm(direction);
        numSteps = ceil(dist/collisionResolution);
        stepVec = direction/numSteps;
    
        bool = true;
        for i = 1:numSteps
            dx = A.position + i * stepVec;
    
            if ~Planner.validState(dx)
                disp('Invalid Edge')
                bool = false;
                return
            end
        end
    end

    function [startNode, goalNode] = randStartGoalNodes(SPheight)  
        validState = 0; 
        while validState == 0 % Rand Start State  
            startPosition = [-100+200*rand(), -100+200*rand(), SPheight+200*rand(), deg2rad(-15+30*rand()), deg2rad(-15+30*rand()), deg2rad(-15+30*rand())];
            startNode = ssspGraphNode(startPosition);
            validState = Planner.validState(startNode);   
        end
    
        validState = 0;
        while validState == 0 % Rand Goal State
            goalPosition = [-100+200*rand(), -100+200*rand(), SPheight+200*rand(), deg2rad(-15+30*rand()), deg2rad(-15+30*rand()), deg2rad(-15+30*rand())];
            goalNode = ssspGraphNode(goalPosition);
            validState = Planner.validState(goalNode);
        end
    end
end
end

%% Notes
% Once using error cost:
% - Dont assume rejection sampling and heuristic conditions can be used.
%


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