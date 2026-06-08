classdef Planner < handle

properties
    SP
    G, kdTree, Q, alg
    metric, c, h   % Metric, Cost, and Heuristic Functions resp.
    steer, dt, stoppingCriteria
    delta, ballConstant, goalBias, tol
    W
    Data, f_write
end

methods
    function obj = Planner(cfg, SP)
        numDims              = SP.DoF;
        obj.SP               = SP;
        obj.G                = motionGraph(numDims, [SP.bounds.minPos, SP.bounds.minRot], [SP.bounds.maxPos, SP.bounds.maxRot], cfg.general.maxNodes);
        obj.Q                = sbbstreeQ(cfg.general.maxNodes/10);  % Self balancing binary search tree (heap)
        obj.alg              = cfg.alg.type;

        wPos                 = norm(SP.bounds.maxPos-SP.bounds.minPos);
        wRot                 = norm(SP.bounds.maxRot-SP.bounds.minRot);
        obj.W                = (wPos/wRot);    % Weight on rotation terms.

        obj.metric           = @(A, B) Planner.distFun(A, B, obj.W);
        obj.c                = @(A,B,varargin) obj.uncertaintyCostFun(A,B,varargin{:});
        obj.h                = @(A, B) 0; %obj.metric(A, B);  % heuristic condition

        obj.kdTree           = KDTree(numDims, obj.metric);

        obj.dt               = 1;
        obj.tol              = cfg.alg.tol;
        obj.delta            = cfg.alg.delta;
        obj.ballConstant     = cfg.alg.ballConstant;
        obj.goalBias         = cfg.alg.goalBias;
        obj.steer            = @SPsteer;

        obj.f_write          = cfg.general.f_write;
        obj.initData(cfg);
        
        switch cfg.general.stopType
            case 'time'
                obj.stoppingCriteria = @(t) t >= cfg.general.maxTime;
            case 'nodes'
                obj.stoppingCriteria = @(t) obj.G.n >= cfg.general.maxNodes;
        end
        
        startNode = cfg.startNode; goalNode = cfg.goalNode;
        startNode.P = cfg.P_start;
        startNode.g = 0; startNode.lmc = 0; startNode.dist = 0;
        obj.G.populateGraphNodes(startNode, goalNode);
        obj.kdTree.kdInsertAsPayload(obj.G.startNode);
        % fprintf('Min dist to goal: %3.f \n', obj.distFun(startNode, goalNode, obj.W))
    end
end
methods(Access = public)
    function trialData = run(obj)
        t_start = tic;
        while ~obj.stoppingCriteria(toc(t_start))
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
            obj.writeData(toc(t_start), obj.G.n, hyperBallRad(obj.G, obj.delta, obj.ballConstant), obj.G.goalNode.g, obj.G.goalNode.dist, trace(obj.G.goalNode.P), obj.G.motionPath)
        end
        obj.interpolateData();
        obj.Data.Tree = obj.G; % Save Full Tree
        obj.Data.alg = obj.alg;
        trialData = obj.Data;        % Return
    end

    function RRT(obj)
        x_rand = ssspGraphNode(obj.G.randomPoint(obj.goalBias));
        while ~obj.validState(x_rand)
            x_rand = ssspGraphNode(obj.G.randomPoint(obj.goalBias));
        end
    
        [x_near, x_near_dist] = kdFindNearestPayload(obj.kdTree, x_rand.position);
        if x_near_dist == 0; return; end
    
        traj = obj.steer(x_near, x_rand, obj.delta, obj.metric);
        x_new = ssspGraphNode(x_near.position + traj, x_near);
        if ~obj.validEdge(x_new, x_near); return; end
    
        [~, x_new.P] = obj.SP.propagateBelief(x_near.position, x_near.P, x_new.position, obj.dt);
        x_new.lmc = x_near.g + obj.c(x_near, x_new);
        x_new.g   = x_new.lmc;
    
        obj.G.insertNode(x_new);
        obj.kdTree.kdInsertAsPayload(x_new);
        obj.updateQueue(x_new);
    
        % Goal detection
        if obj.metric(x_new, obj.G.goalNode) < obj.tol && ...
                obj.validEdge(x_new, obj.G.goalNode)
            [~, testP] = obj.SP.propagateBelief(x_new.position, x_new.P, obj.G.goalNode.position, obj.dt);
            if x_new.g + obj.uncertaintyCostFun(x_new, obj.G.goalNode, testP) < obj.G.goalNode.g
                obj.G.goalNode.lmc   = x_new.g + obj.c(x_new, obj.G.goalNode, testP);
                obj.G.goalNode.g     = obj.G.goalNode.lmc;
                obj.G.goalNode.P     = testP;
                obj.G.goalNode.parent = x_new;
                fprintf('New Solution Found: %.3f\n', obj.G.goalNode.g);
            end
        end
    end

    function RRTstar(obj)
        x_rand = ssspGraphNode(obj.G.randomPoint(obj.goalBias));
        while ~obj.validState(x_rand) %|| ... % Rej. Sampling
               % Planner.distFun(x_rand, obj.G.startNode, obj.W) + Planner.distFun(x_rand, obj.G.goalNode, obj.W) >= obj.G.goalNode.g % Rejection Sampling
            x_rand = ssspGraphNode(obj.G.randomPoint(obj.goalBias));
        end
        x_new = obj.extend(x_rand);
        if ~isempty(x_new); x_new.g = x_new.lmc; end
        
        obj.G.goalNode.g = obj.G.goalNode.lmc; % Allows for common extend fun. between RRT* and RRT#
    end
    
    function RRTsharp(obj)
        x_rand = ssspGraphNode(obj.G.randomPoint(obj.goalBias));
        while ~obj.validState(x_rand) %|| ...
                %Planner.distFun(x_rand, obj.G.startNode, obj.W) + Planner.distFun(x_rand, obj.G.goalNode, obj.W) >= obj.G.goalNode.g % Rejection Sampling
            x_rand = ssspGraphNode(obj.G.randomPoint(obj.goalBias));
        end
        obj.extend(x_rand);
        obj.replan();
    end

    function x_new = extend(obj, x_rand)
        [x_near, x_near_dist] = kdFindNearestPayload(obj.kdTree, x_rand.position);
        if x_near_dist == 0; x_new = []; return; end               % Avoids resampling existing nodes
        traj = obj.steer(x_near, x_rand, obj.delta, obj.metric);
        x_new = ssspGraphNode(x_near.position + traj, x_near);
        if ~obj.validEdge(x_new, x_near); x_new = []; return; end  % Verify possible edge
        [~, x_new.P] = obj.SP.propagateBelief(x_near.position, x_near.P, x_new.position, obj.dt); % Check later to make sure pos match
        x_new.lmc = x_new.parent.g + obj.c(x_new.parent, x_new); 
        
        range = hyperBallRad(obj.G, obj.delta, obj.ballConstant);
        neighbors = kdFindWithinRange(obj.kdTree, range, x_new.position);

        for n_i = 1:length(neighbors)
            neighbor = neighbors{n_i}.payload;
            if obj.validEdge(x_new, neighbor)
                [~, testP] = obj.SP.propagateBelief(neighbor.position, neighbor.P, x_new.position, obj.dt);
                if x_new.lmc > neighbor.g + obj.c(neighbor, x_new, testP)    
                    x_new.lmc = neighbor.g + obj.c(neighbor, x_new, testP);
                    x_new.parent = neighbor;
                end
                x_new.successors{end + 1, 1} = neighbor;
                neighbor.successors{end + 1, 1} = x_new;
            end
        end
    
        obj.G.insertNode(x_new);
        obj.kdTree.kdInsertAsPayload(x_new);
        obj.updateQueue(x_new);
        
        if obj.metric(x_new, obj.G.goalNode)  < obj.tol && ...
                obj.validEdge(x_new, obj.G.goalNode)
            [~, testP] = obj.SP.propagateBelief(x_new.position, x_new.P, obj.G.goalNode.position, obj.dt);
            if x_new.lmc + obj.uncertaintyCostFun(x_new, obj.G.goalNode, testP) < obj.G.goalNode.g
                obj.G.goalNode.P = testP;
                obj.G.goalNode.lmc = x_new.lmc + obj.c(x_new, obj.G.goalNode, testP);
                obj.G.goalNode.parent = x_new;
                x_new.successors{end + 1, 1} = obj.G.goalNode;
                fprintf('New Solution Found: %.3f \n', obj.G.goalNode.lmc);
                obj.G.goalNode.dist = x_new.dist + obj.distFun(x_new, obj.G.goalNode, obj.W);
                obj.updateQueue(obj.G.goalNode);
            end
        end
    end

    function replan(obj)
        while ~obj.Q.isEmpty() && topKey(obj.Q) <= min(obj.G.goalNode.g, obj.G.goalNode.lmc)
            x = obj.Q.pop();
            x.g = x.lmc;

            for n_i = 1:length(x.successors)
                successor = x.successors{n_i};
                [~, testP] = obj.SP.propagateBelief(x.position, x.P, successor.position, 1);
                cost = x.g + obj.c(x, successor, testP);
    
                if successor.lmc > cost
                    successor.lmc = cost;
                    successor.parent = x;
                    successor.dist = x.dist + obj.distFun(x, successor, obj.W);
                    obj.updateQueue(successor);

                    if successor == obj.G.goalNode
                        fprintf('New Solution Found: %.3f (Replan)\n', cost);
                    end
                end
            end
        end
    end

    function bool = validState(obj, x)
        % Accepts both ssspGraphNode and numeric position vector
    
        if isnumeric(x)
            pos = x;
        elseif isprop(x, 'position')
            pos = x.position;
        else
            error('validState: input must be a node or numeric position vector');
        end
    
        l_actuators = obj.SP.inverseKinematics(pos);
        bool = LinearActuator.actCheck(l_actuators);
        % if bool == false; disp('Invalid State'); end % DEBUG
    end

    function [bool] = validEdge(obj, A, B)
        collisionResolution = 10; % mm
        if A.position == B.position; bool = true; return; end % If coinciding nodes: break, return true

        direction = B.position - A.position;
        dist = norm(direction);
        numSteps = ceil(dist/collisionResolution);
        stepVec = direction/numSteps;
    
        bool = true;
        for i = 1:numSteps
            dx = A.position + i * stepVec;
    
            if ~obj.validState(dx)
                % disp('Invalid Edge') % DEBUG
                bool = false;
                return
            end
        end
    end
end

methods(Access = private)
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

    function cost = uncertaintyCostFun(obj, A, B, B_P)
        % If covariance explicitly passed (for rewiring case)
        if nargin < 4 || isempty(B_P); B_P = B.P; end      
        % Trace Cost:
        cost = trace(B_P)*obj.distFun(A, B, obj.W);
        
        % Volumetric Cost:
        % n = size(B_P,1);
        % V = (pi^(n/2) / gamma(n/2 + 1)) * sqrt(det(B_P));
        % cost = V * obj.distFun(A, B, obj.W);

    end

    function initData(obj, cfg)
        maxTime = cfg.general.maxTime;
        numCol = maxTime*obj.f_write;
        obj.Data = struct();
        obj.Data.Time = NaN(numCol, 1);
        obj.Data.NumNodes = NaN(numCol, 1);
        obj.Data.BallRad = NaN(numCol,1);
        obj.Data.Cost = NaN(numCol, 1);
        obj.Data.PathDist = NaN(numCol, 1);
        obj.Data.GoalUncertainty = NaN(numCol, 1);
        obj.Data.Path = cell(numCol, 1);
        obj.Data.Tree = cell(1);
    end

    function writeData(obj, t_elapsed, numNodes, ballRad, currentCost, pathDist, currentUncertainty, currentPath)
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
            obj.Data.Time(slot_idx)            = t_elapsed;       % actual time
            obj.Data.NumNodes(slot_idx)        = numNodes;
            obj.Data.BallRad(slot_idx)         = ballRad;
            obj.Data.Cost(slot_idx)            = currentCost;
            obj.Data.PathDist(slot_idx)        = pathDist;
            obj.Data.GoalUncertainty(slot_idx) = currentUncertainty;
            obj.Data.Path{slot_idx}            = currentPath;
        end
    end

    function interpolateData(obj)
        % Any cell that is NaN due to skipped writing copies previous cell
        for i = 2:length(obj.Data.Time)
            if isnan(obj.Data.Time(i));            obj.Data.Time(i) =            obj.Data.Time(i - 1);    end
            if isnan(obj.Data.NumNodes(i));        obj.Data.NumNodes(i) =        obj.Data.NumNodes(i - 1);end
            if isnan(obj.Data.BallRad(i));         obj.Data.BallRad(i) =         obj.Data.BallRad(i - 1); end
            if isnan(obj.Data.Cost(i));            obj.Data.Cost(i) =            obj.Data.Cost(i - 1);    end
            if isnan(obj.Data.PathDist(i));        obj.Data.PathDist(i) =        obj.Data.PathDist(i - 1);end
            if isnan(obj.Data.GoalUncertainty(i)); obj.Data.GoalUncertainty(i) = obj.Data.GoalUncertainty(i - 1);end
            if isnan(obj.Data.Path{i});            obj.Data.Path{i} =            obj.Data.Path{i - 1};    end
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
end
end

%% Notes
% -Volumetric cost function is implemented but commented out, must
% create option in config for this.