clear variables;
% Positions:
SP.height = 326.2957;
StartSet = zeros(10,6);
GoalSet = zeros(10,6);
 for i = 1:5
 check = 1;
    while check == 1
        
        startPosition = [-100+200*rand(), -100+200*rand(), SP.height+200*rand(), deg2rad(-15+30*rand()), deg2rad(-15+30*rand()), deg2rad(-15+30*rand())];
        check = PosCheck(startPosition);
        
    end
    StartSet(i,:) = startPosition;
    check = 1;
    while check == 1 
        
        goalPosition = [-100+200*rand(), -100+200*rand(), SP.height+200*rand(), deg2rad(-15+30*rand()), deg2rad(-15+30*rand()), deg2rad(-15+30*rand())];
        check = PosCheck(goalPosition);
        
    end
    GoalSet(i,:) = goalPosition;
 end
 
 % Extremes:
 % Bottom to Top:
StartSet(6,:) = [-100, -100, SP.height+5, deg2rad(5), deg2rad(5), deg2rad(5)];
GoalSet(6,:) = [100, 100, SP.height+160, deg2rad(-5), deg2rad(-5), deg2rad(-5)];
 
 %Top to Bottom:
StartSet(7,:) = [-100, 100, SP.height+160, deg2rad(5), deg2rad(5), deg2rad(5)];
GoalSet(7,:) = [100, -100, SP.height+5, deg2rad(-10), deg2rad(5), deg2rad(-10)];
 
 % Vertical Motion with Large Rotation:
StartSet(8,:) = [0, 0, SP.height+50, deg2rad(-15), deg2rad(-15), deg2rad(-15)];
GoalSet(8,:) = [0, 0, SP.height+200, 0, 0, 0];
 
 % Large Translation on same height:
StartSet(9,:) = [100, -100, SP.height+100, deg2rad(15), deg2rad(-15), deg2rad(15)];
GoalSet(9,:) = [-100, 100, SP.height+100, deg2rad(-15), deg2rad(15), deg2rad(-15)];
 
 % Pure straight Line Translation:
StartSet(10,:) = [0, -100, SP.height+100, deg2rad(0), deg2rad(0), deg2rad(0)];
GoalSet(10,:) = [0, 100, SP.height+100, deg2rad(0), deg2rad(0), deg2rad(0)]; 

NumTrials = 5;
Data(NumTrials).Path.IT = nan(100,1);
Data(NumTrials).Err.IT = cell(100,1);
Data(NumTrials).Time = NaN(100,1);

Data(NumTrials).NumNodes = NaN(100,1);
set(0,'DefaultFigureVisible','off');
%{
%SP.height = 326.2957;
problemDefinition = 1;
if problemDefinition == 1
    %{
    check = 1;
    while check == 1
        
        startPosition = [-100+200*rand(), -100+200*rand(), SP.height+200*rand(), deg2rad(-15+30*rand()), deg2rad(-15+30*rand()), deg2rad(-15+30*rand())];
        check = PosCheck(startPosition);
        
    end
    check = 1;
    while check == 1 
        
        goalPosition = [-100+200*rand(), -100+200*rand(), SP.height+200*rand(), deg2rad(-15+30*rand()), deg2rad(-15+30*rand()), deg2rad(-15+30*rand())];
        check = PosCheck(goalPosition);
        
    end
    %}
    tend = 30*60;
    startPosition = [90.5533,-20.7020,500.4588,-0.0728,0.1591,0.1691];
    goalPosition = [85.4063,-60.7302,464.1589,0.1198,-0.0381,-0.1102];
elseif problemDefinition == 2
    %{
    check = 1;
    while check == 1
        [startPosition,check] = initState([200*rand(); 200*rand()]);
        startPosition = startPosition';
    end
    check = 1;
    while check == 1
        [goalPosition,check] = initState([200*rand(); 200*rand()]);
        goalPosition = goalPosition';
    end
    %}
   startPosition = [31.5430, 728.0762];
    goalPosition = [12.4457, 657.6901];
    tend = 2*60;
end
jj = 1; % movie count
%}
%%
for EFV = 1 %1:2
for Count = 1 %1:10
    QVal = 1;
    RVal = 1;
    clear Data
set(0,'DefaultFigureVisible','off');
for TN = 1:NumTrials
    clearvars -except TN Data NumTrials StartSet GoalSet tend RVals QVals QVal RVal Count EFV jj
    close all;
    dbstop if error
    warning('off', 'MATLAB:bvp4c:RelTolNotMet');   % turn off this particular warning about convergence tolerances not being met (we just throw out points of which this happens)
    % these are the start/goal positions (note that they are reset
    % to the closest nodes in the graph if the graph is loaded
    % from a file), if instead we want to use a node ID, then
    % that can be done with the following option
    problemDefinition = 1;
    startPosition = StartSet(Count,:);
    goalPosition = GoalSet(Count,:);
    tend = 30*60;
    % problemDefinition = 2;  % 1: Stewart Platform [x,y,z,Xrot,Yrot,Zrot]
                            % 2: 2D Example [x,y]
    %------------------------- BEGIN USER INPUT ------------------------------%
    if problemDefinition == 2
        Tol = 0.1;              % Goal position tolerance
    else
        Tol = 1;
    end
    algorithmToUse = 2;     % 0: RRT
                            % 1: RRT*
                            % 2: RRT#
    SP.height = 326.2957;   % Base height from bottom plate to top plate [mm]
    
    drawEachStep = false;   % Set to true to draw each step when plotting.
    maxNodes = 3000;       % Max number of nodes in the graph structure.
    goalBias = 0.05;        % This fraction of the time sample the goal
    Loop = 0;
    % tend = 3*60;
    %%% If using obstacles, add initial stuff here.
     
    
    if problemDefinition == 1
        numDims = 6;    % Number of dimensions in the C-space
        
        
        % fprintf('Trial Num %2i\n', TrialNum);
        %{
        % Start and Goal Positions:
        check = 1;
        while check == 1
            startPosition = [-100+200*rand(), -100+200*rand(), SP.height+200*rand(), deg2rad(-15+30*rand()), deg2rad(-15+30*rand()), deg2rad(-15+30*rand())];
            check = PosCheck(startPosition);
        end
        check = 1;
        while check == 1
            goalPosition = [-100+200*rand(), -100+200*rand(), SP.height+200*rand(), deg2rad(-15+30*rand()), deg2rad(-15+30*rand()), deg2rad(-15+30*rand())];
            check = PosCheck(goalPosition);
        end
        %}
        
        % Dimension Settings:
        dimensionMins = [-100, -100, SP.height, deg2rad(-15), deg2rad(-15), deg2rad(-15)];
        dimensionMaxs = [100, 100, 200+SP.height, deg2rad(15), deg2rad(15), deg2rad(15)];
        
        % Position Check:
        PC = @PosCheck;
        
        wPos = norm(dimensionMaxs(1:3)-dimensionMins(1:3));
        wRot = norm(dimensionMaxs(4:6)-dimensionMins(4:6));
        W = (wPos/wRot); % Weight on Rotation terms in distance functions
        % W = 1;
        
        delta = 10;         % Distance to expand tree toward random point
        ballConstant = 25; % Affects the size of the relative neighborhood
        
        if EFV == 1
            ErrFun = @VolEllipsoid2;
        elseif EFV == 2
            ErrFun = @(PMat) trace(PMat);
        end
        steer = @SPSteerV4; % steering function to use
        costAdd = @(PosCost,RotCost) (PosCost+W*RotCost);
        CostCompare = @(PosLHS,RotLHS,PosRHS,RotRHS) (PosLHS < PosRHS && RotLHS < RotRHS);
        
        % Set up Q Matrix wanted:
        Poserr = 2;          % Assuming an initial position error of 2 mm
        Angerr = deg2rad(2); % Assuming an initial angle error of 1 degrees
        QMat = QVal*[Poserr*eye(3) 1e-6*ones(3); 1e-6*ones(3) Angerr*eye(3)];
        
    elseif problemDefinition == 2
        numDims = 2;
        
        check = 1;
        %{
        while check == 1
            [startPosition,check] = initState([200*rand(); 200*rand()]);
            startPosition = startPosition';
        end
        check = 1;
        while check == 1
            [goalPosition,check] = initState([200*rand(); 200*rand()]);
            goalPosition = goalPosition';
        end
        %}
        
        
        % The min and max dimensions based on the tested range of the
        % actuators, however they include positions impossible to be reached,
        % therefore a check function is needed.
        dimensionMins = [-145, 490];
        dimensionMaxs = [145, 790];
        
        % Point check function:
        PC = @PosCheck2D;
        
        % Error calculation Function:
        if EFV == 1
            ErrFun = @VolEllipsoid;
            
        elseif EFV == 2
            ErrFun = @(PMat) trace(PMat);
        end
        
        W = 1; % No weighting in this case, just needed for continuity
        delta = 10;
        ballConstant = 200;
        steer = @Steer2D_V4;
        
        QMat = QVal*[0.0844 -5.0781e-5; -5.0781e-5 0.0844]; % Process Noise Covariance
        
    end
    
    %--------------------------- END USER INPUT ------------------------------%
    % Set up stopping Criteria function:
    stoppingCriteriaType = algorithmToUse;
    
    if stoppingCriteriaType == 0
        % Assumes forward search, and that we set cost to start to be non-inf
        % when it is connected to the graph (used with RRT).
        stopingCriteriaMet = @(graph) (~isinf(norm(graph.goalNode.err)) || graph.n >= graph.maxNodesAllowed);
        % stopingCriteriaMet = @(graph) (graph.n >= graph.maxNodesAllowed);
        
    elseif stoppingCriteriaType == 1
        % Assumes forward search, and that we just go until the max number of
        % nodes is reached.
        % stopingCriteriaMet = @(graph)  (graph.n >= graph.maxNodesAllowed);
        stopingCriteriaMet =@(tcur) (tcur >= tend);
        
    elseif stoppingCriteriaType == 2
        % Assumes forward search, and that we just go until the max number of
        % nodes is reached.
        % stopingCriteriaMet = @(graph)  (graph.n >= graph.maxNodesAllowed);
        stopingCriteriaMet =@(tcur) tcur >= tend;
    else
        error('unknown stopping criteria function')
    end
    
    
    % Set up graph structure:
    G = motionGraph();
    
    populateForSBSearch(G,  delta, dimensionMins, dimensionMaxs, ballConstant, startPosition, goalPosition, maxNodes, numDims);
    
    if problemDefinition == 1
        % Set up parameters of topology for KD-tree and other functions:
        kdTreeDistFunct = @(inputA, inputB) norm(inputA(1:3)-inputB(1:3))+W*norm(inputA(4:6)-inputB(4:6));
        kdTree = KDTree(numDims, kdTreeDistFunct);
        
        % No wrapping dimensions so none need to be set up (otherwise do it here).
        
        % Set up trajectory distance function:
        trajectoryLength = @TrajLength;
        
        
        
        % Remember axis limits for plotting:
        ax_bds = [dimensionMins(1:3) rad2deg(dimensionMins(4:6)); dimensionMaxs(1:3) rad2deg(dimensionMaxs(4:6))];
        ax_bds = ax_bds(:)';
    elseif problemDefinition == 2
        
        
        kdTreeDistFunct = @(inputA, inputB) norm(inputA-inputB);
        kdTree = KDTree(numDims, kdTreeDistFunct);
        trajectoryLength = @TrajLength2D;
        
        ax_bds = [dimensionMins; dimensionMaxs];
        ax_bds = ax_bds(:)';
        
    end
    
    tstart = tic;           % start timing
    tcur = 0;
    Q = sbbstreeQ(maxNodes/10);    % Self balancing binary search tree (heap),
                                    % this will grow if necessary
                                    % NOTE: that because this implementation has
                                    % only a singleton key and not a tupal key,
                                    % the resulting algorithm may not work well
                                    % in euclidian spaces on grids (or other
                                    % places where cost value collisions are
                                    % likely), it should be fine for the 2pbvp
                                    % case here.
    % ----------------- Start of algorithm -----------------
    
    % Mark cost to start:
    G.startNode.err = ErrFun(G.startNode.PMat);
    % G.startNode.travFromStart = 0;
    G.goalNode.err = inf;
    % G.goalNode.travFromStart = inf;
    % Insert start and goal into the KD tree. Note that given reuse of sorting
    % data structures with similarily named fields, we use different node types
    % for the KD tree than for the graph. The KD tree nodes will hold a handle
    % to the graph node in their (the KD tree node's) payload field. This is
    % why we use the payload versions of the kdtree functions.
    
    kdInsertAsPayload(kdTree, G.startNode);
    goalInKDTree = false;
    goalErr = inf;
    %while ~stopingCriteriaMet(G)
    while ~stopingCriteriaMet(tcur)
        
        % Get random point, occasionally sample the goal
        usingGoal = false;
        if rand() < goalBias
            % Use the goal node
            randPoint = G.goalNode.position;
            usingGoal = true;
        else
            GoodPoint = 1;
            while GoodPoint == 1
                % sample a random point
                randPoint = randomPoint(G);
                GoodPoint = PC(randPoint'); % Position Check random point
            end
        end
        
        % Find the nearest point that is already in the graph
        [closestNode, distToclosestNode] = kdFindNearestPayload(kdTree, randPoint);
        
        % Now sample a new point found by steering from closestNode toward randPoint
        % note that the following function will return a local path from
        % closestNode's position to the new point
        [localTrajectory, newPMat] = steer(closestNode.position', randPoint', G.delta,W,Tol, closestNode.PMat, QMat, RVal);
        if isempty(localTrajectory)
            %   warning('could not connect')
            continue
        end
        if isnan(newPMat)
            continue
        elseif isnan(localTrajectory)
            continue
        end
        
        newPoint = localTrajectory(end, :);
        localErr = ErrFun(newPMat);
        errFromStartNewPoint = localErr + closestNode.err;
        
        % Taking Error as the new Cost:
        
        parentNode = closestNode;                % we will change later if we find a better one (if using RRT*, etc)
        % localTrav = trajectoryLength(localTrajectory) + parentNode.travFromStart;
        
        if algorithmToUse == 0
            % running RRT
            % we only need to consider one node for parent
            
        elseif algorithmToUse == 1
            % Running RRT*
            range = hyperBallRad(G);   % radius of shrinking D-Ball
            disp(['hyperball rad: ' num2str(range) ', nodes in graph: ' num2str(G.n)])
            
            kNN = kdFindWithinRange(kdTree, range, parentNode.position);
            numNeighbors = length(kNN);
            
            for n_i = 1:numNeighbors
                neighborNode = kNN{n_i}.payload;
                
                if closestNode.id == neighborNode.id
                    % we've already calculated trajectory from this node to new
                    % point (it was the first thing in localTrajectory)
                    continue
                    % elseif kdTreeDistFunct(neighborNode.position, newPoint) + neighborNode.travFromStart < localTrav
                elseif neighborNode.err < errFromStartNewPoint
                    % the above check is a heuristic check, if we are here than
                    % the neighbor node MAY be better, but now we need to do the
                    % harder actual 2pbvp check to make sure
                    
                    
                    [neighborLocalTrajectory,neighborPMat] = steer(neighborNode.position', newPoint', delta,W,Tol,neighborNode.PMat, QMat, RVal);
                    if isempty(neighborLocalTrajectory)
                        % warning('could not connect from neighbor')
                        continue
                    end
                    if isnan(neighborPMat)
                        continue
                    elseif isnan(neighborLocalTrajectory)
                        continue
                    end
                    
                    % thisTrav = trajectoryLength(neighborLocalTrajectory);
                    thisErr = ErrFun(neighborPMat);
                    tolCheck = norm(neighborLocalTrajectory(end,:)-newPoint);
                    
                    if thisErr+neighborNode.err < errFromStartNewPoint && tolCheck <= Tol
                        % we have found a better parent
                        
                        localErr = thisErr;
                        errFromStartNewPoint = neighborNode.err + thisErr;
                        newPMat = neighborPMat;
                        localTrajectory = neighborLocalTrajectory;
                        % localTrav = thisTrav + neighborNode.travFromStart;
                        parentNode = neighborNode;
                    end
                    
                end
            end
            
            % at this point we have found the best neighbor in the set to use
            % as a parent
        elseif algorithmToUse == 2
            % Running RRT#
            % (we will save all neighbors as part of the graph)
            % we take into account actual trajectory length for all nodes
            % within the shrinking D-ball
            
            range = hyperBallRad(G);   % radius of shrinking D-Ball
            disp(['hyperball rad: ' num2str(range) ', nodes in graph: ' num2str(G.n)])
            
            kNN = kdFindWithinRange(kdTree, range, parentNode.position);
            numPredecessors = length(kNN);
            
            % unlike the slighly sloppy (but slightly faster) way we did it
            % for RRT*, here for RRT# will will explicitly name the best
            % trajectories as we loop over the neighbor set
            % at this point we search for predecessor neighbors
            
            bestErr = Inf;
            bestPredecessorIndex = -1;
            
            predecessorNodes = cell(numPredecessors,1);
            trajectoriesFromPredecessors = cell(numPredecessors,1);
            % travFromPredecessor = inf(numPredecessors,1);
            PMatFromPredecessor = cell(numPredecessors,1);
            errFromPredecessor = inf(numPredecessors,1);
            numValidPredecessors = 0;  % incriments up as neighbors are added
            
            for n_i = 1:numPredecessors
                predecessorNode = kNN{n_i}.payload;
                
                if closestNode.id == predecessorNode.id
                    % we've already calculated trajectroy from this node to new
                    % point (it was the first thing in localTrajectory)
                    % so just copy the following values over
                    predecessorLocalTrajectory = localTrajectory;
                    thisErr = localErr;
                    thisPMat = newPMat;
                    
                else
                    %  now we need to do the 2pbvp check to make sure there is
                    %  a valid trajectory from this node to its neighbor
                    
                    
                    [predecessorLocalTrajectory,predecessorPMat] = steer(predecessorNode.position', newPoint', delta,W,Tol, predecessorNode.PMat, QMat, RVal);
                    if isempty(predecessorLocalTrajectory)
                        % warning('could not connect from neighbor')
                        continue
                    end
                    if isnan(predecessorPMat)
                        continue
                    elseif isnan(predecessorLocalTrajectory)
                        continue
                    end
                    
                    
                    thisPMat = predecessorPMat;
                    thisErr = ErrFun(predecessorPMat);
                end
                
                % now we add this incomming edge (from the predessor neighbor)
                % to a temporary list that we will use later if this becomes an
                % actual node in the graph
                numValidPredecessors = numValidPredecessors + 1;
                predecessorNodes{numValidPredecessors,1} = predecessorNode;
                trajectoriesFromPredecessors{numValidPredecessors,1} = predecessorLocalTrajectory;
                errFromPredecessor(numValidPredecessors,1) = thisErr;
                PMatFromPredecessor{numValidPredecessors,1} = thisPMat;
                
                tolCheck = norm(predecessorLocalTrajectory(end,:)-newPoint); % Check to make sure actually reaching the new Point
                
                if thisErr + min(predecessorNode.err,predecessorNode.lmc) < bestErr && tolCheck <= Tol
                    % we have found a better parent
                    
                    bestErr = localErr + min(predecessorNode.err,predecessorNode.lmc);
                    bestPredecessorIndex = numValidPredecessors;
                end
            end
            
            % now store the parent information
            if numValidPredecessors <= 0
                error('had a connection but failed to find it in neighbor search')
            end
            
            if bestPredecessorIndex < 0
                % no parent was found, so just continue
                error('had a connection but failed to find a best predecessor')
            end
            
            
        end
        goalError = kdTreeDistFunct(newPoint,G.goalNode.position);
        if goalError < goalErr
            goalErr = goalError;
        end
        if goalErr < Tol && goalInKDTree == false
            
            % if we just got back the goal's position
            newNode = G.goalNode;
            if ~goalInKDTree
                kdInsertAsPayload(kdTree, newNode);
                goalInKDTree = true;
                disp('Goal in KD Tree')
            end
        else
            
            % insert a new graph node at that point and get a handle to it
            newNode = insertNewNodeForSBSearch(G, newPoint);
            
            % also insert that node into the kd-tree
            kdInsertAsPayload(kdTree, newNode);
        end
        
        if algorithmToUse == 0 || algorithmToUse == 1
            % running RRT or RRT*
            % make the closest node the parent of the new node
            
            if parentNode.id == newNode.id
                % don't make a node a parent of itself
                continue
            end
            
            newNode.parentID = parentNode.id;
            newNode.parentNode = parentNode;
            newNode.parentTraj = localTrajectory;
            % newNode.travFromStart = localTrav;
            newNode.err = localErr + parentNode.err;
            newNode.PMat = newPMat;
            
        elseif algorithmToUse == 2
            % running RRT#
            
            % now add the "from" neighbors and neighbor trajectories (recall
            % we're doing forward search, so "from" neighbors are the predessors we
            % follow back to the start, and "to" neighbors are the successors that
            % might ba able to use this node as a parent)
            
            if newNode.id == G.goalNode.id
                % the goal is the tricky case, since it already has some of
                % these neighbors stored already
                
                % first figure out which of the newly found neighbors are
                % actually new neighbors
                predecessorInds = inf(length(newNode.predecessors) + numValidPredecessors,1);
                
                for n_i = 1:length(newNode.predecessors)
                    predecessorInds(n_i) = newNode.predecessors{n_i}.id;
                end
                for n_i = 1:numValidPredecessors
                    if predecessorNodes{n_i}.id == G.goalNode.id
                        % we also don't want to use the goal as its own predecessor
                        continue
                    end
                    predecessorInds(length(newNode.neighbors) + n_i) = predecessorNodes{n_i}.id;
                end
                [junk, uniqueIndsInds, alsoJunk] = unique(predecessorInds, 'stable');
                
                % the following will remove all indicies associated with old
                % predecessors in the goal (keeping only the indicies of the
                % new predecessors)
                uniqueIndsInds = uniqueIndsInds(uniqueIndsInds > length(newNode.predecessors)) - length(newNode.predecessors);
                
                % now finally copy over the new ones
                oldLength = length(newNode.predecessors);
                newNode.predecessors = [newNode.predecessors ; predecessorNodes(uniqueIndsInds,1)];
                newNode.errFromPredecessors = [newNode.errFromPredecessors ; errFromPredecessor(uniqueIndsInds,1)];
                newNode.PMatFromPredecessors = [newNode.PMatFromPredecessors ; PMatFromPredecessor(uniqueIndsInds,1)];
                newNode.trajectoriesFromPredecessors = [newNode.trajectoriesFromPredecessors ; trajectoriesFromPredecessors(uniqueIndsInds,1)];
                
                % update the indicies to reflect the new predecessors actually stored in the node's list
                uniqueIndsInds = oldLength+1:length(newNode.predecessors);
                
                % now we need to see if any of the new nodes yield a better cost from start
                bestPredecessorIndex = -1;
                bestErrFromStart = inf;
                
                for n_i = 1:length(newNode.predecessors)
                    thisPredecessor = newNode.predecessors{n_i};
                    thisErrFromStart = thisPredecessor.err + newNode.errFromPredecessors(n_i,1);
                    
                    tolCheck = norm(newNode.trajectoriesFromPredecessors{n_i,1}(end,:)-newNode.position);
                    if thisErrFromStart < bestErrFromStart && tolCheck <= Tol
                        bestErrFromStart = thisErrFromStart;
                        bestPredecessorIndex = n_i;
                    end
                end
                
                
                if  bestPredecessorIndex > 0
                    % only update the goal's parent if it actually helps
                    parentNode = newNode.predecessors{bestPredecessorIndex,1};
                    
                    newNode.lmc = bestErrFromStart;
                    newNode.parentID = parentNode.id;
                    newNode.parentNode = parentNode;
                    newNode.parentTraj = newNode.trajectoriesFromPredecessors{bestPredecessorIndex,1};
                    newNode.PMat = newNode.PMatFromPredecessors{bestPredecessorIndex,1};
                    
                end
                
            else
                
                parentNode = predecessorNodes{bestPredecessorIndex};
                
                
                % for normal nodes, we just init these things as required
                newNode.lmc = errFromPredecessor(bestPredecessorIndex) + parentNode.err;
                newNode.parentID = parentNode.id;
                newNode.parentNode = parentNode;
                newNode.predecessors = predecessorNodes(1:numValidPredecessors);
                newNode.errFromPredecessors = errFromPredecessor(1:numValidPredecessors,1);
                newNode.PMatFromPredecessors = PMatFromPredecessor(1:numValidPredecessors,1);
                newNode.trajectoriesFromPredecessors = trajectoriesFromPredecessors(1:numValidPredecessors,1);
                newNode.parentTraj = trajectoriesFromPredecessors{bestPredecessorIndex,1};
                newNode.PMat = PMatFromPredecessor{bestPredecessorIndex,1};
                
                uniqueIndsInds = 1:numValidPredecessors;
                
            end
            
            
            % now we need to update the successor lists of all these predessors
            % to include the new node and trajectory
            
            for n_i_i = 1:length(uniqueIndsInds)
                n_i = uniqueIndsInds(n_i_i);
                predecessorNode = newNode.predecessors{n_i};
                
                numValidSuccessors = length(predecessorNode.neighbors) + 1;
                
                
                % this is a place for potential speed improvement, since right
                % now we're growing these on the fly without any explicit
                % prealocation of memory
                predecessorNode.neighbors{numValidSuccessors,1} = newNode;
                predecessorNode.errToNeighbors(numValidSuccessors,1) = newNode.errFromPredecessors(n_i,1);
                predecessorNode.trajectoriesToNeighbors{numValidSuccessors,1} = newNode.trajectoriesFromPredecessors{n_i,1};
                predecessorNode.PMatToNeighbors{numValidSuccessors,1} = newNode.PMatFromPredecessors{n_i,1};
            end
            
        end
        
        if algorithmToUse == 1
            % we are running RRT*
            % we need to adds the reverse edges to the graph
            for n_i = 1:length(kNN)
                
                neighborNode = kNN{n_i}.payload;
                
                if neighborNode.id == newNode.parentNode.id || neighborNode.id == newNode.id
                    % we do not want to rewire the new node or its parent
                    continue
                end
                
                % we do want to see if we would like to rewire these other
                % neighbors to use the new node as their parent
                
                
                if newNode.err < neighborNode.err
                    % the above check is a heuristic check, if we are here than
                    % the neighbor node MAY do better to use the new node as
                    % its parent, but now we need to do the harder actual 2pbvp
                    % check to make sure
                    
                    
                    [neighborLocalTrajectory,neighborPMat] = steer(newNode.position', neighborNode.position', delta, W, Tol, newNode.PMat, QMat, RVal);
                    if isempty(neighborLocalTrajectory)
                        % warning('could not connect to neighbor')
                        continue
                    end
                    if isnan(neighborPMat)
                        continue
                    elseif isnan(neighborLocalTrajectory)
                        continue
                    end
                    tolCheck = norm(neighborLocalTrajectory(end,:)-neighborNode.position);
                    % thisTrav = trajectoryLength(neighborLocalTrajectory);
                    thisErr = ErrFun(neighborPMat);
                    if thisErr+newNode.err < neighborNode.err && tolCheck <= Tol
                        % the new node is a better parent to the neighbor than it
                        % current parent
                        
                        neighborNode.parentID = newNode.id;
                        neighborNode.parentNode = newNode;
                        neighborNode.parentTraj = neighborLocalTrajectory;
                        % neighborNode.travFromStart = thisTrav + newNode.travFromStart;
                        neighborNode.err = thisErr + newNode.err;
                        neighborNode.PMat = neighborPMat;
                    end
                    
                end
                
            end
        elseif algorithmToUse == 2
            % running RRT#
            % need to add reverse edges to the graph
            
            if newNode.id ~= G.goalNode.id
                % we'll take the lazy approach and just say that no nodes will
                % care to direcly use the goal node as their parent
                % (checking this would be slighly more involved since it would
                % tend to accumulate neighbors)
                
                % store these temproarily here, for building, then copy to node later
                numSuccessors = length(kNN);
                sucessorNodes = cell(numSuccessors,1);
                trajectoriesToSuccessors = cell(numSuccessors,1);
                errToSuccessor = inf(numSuccessors,1);
                PMatToSuccessor = cell(numSuccessors,1);
                numValidSuccessors = 0;  % incriments up as neighbors are added
                
                
                for n_i = 1:length(kNN)
                    
                    neighborNode = kNN{n_i}.payload;
                    
                    if neighborNode.id == newNode.id
                        % we do not want to rewire the new node to itself
                        % (though, under normal circumstance I do not think
                        % this case actually happens)
                        continue
                    end
                    
                    [neighborLocalTrajectory,neighborPMat] = steer(newNode.position' , neighborNode.position', delta,W,Tol,newNode.PMat, QMat, RVal);
                    if isempty(neighborLocalTrajectory)
                        % warning('could not connect to neighbor')
                        continue
                    end
                    if isnan(neighborPMat)
                        continue
                    elseif isnan(neighborLocalTrajectory)
                        continue
                    end
                    tolCheck = norm(neighborLocalTrajectory(end,:)-neighborNode.position);
                    if tolCheck <= Tol
                        thisErr = ErrFun(neighborPMat);
                        thisPMat = neighborPMat;
                        % thisTrav = trajectoryLength(neighborLocalTrajectory);
                        
                        numValidPredecessors = length(neighborNode.predecessors) + 1;
                        neighborNode.predecessors{numValidPredecessors,1} = neighborNode;
                        neighborNode.errFromPredecessors(numValidPredecessors,1) = thisErr;
                        neighborNode.PMatFromPredecessors{numValidPredecessors,1} = thisPMat;
                        neighborNode.trajectoriesFromPredecessors{numValidPredecessors,1} = neighborLocalTrajectory;
                        
                        
                        numValidSuccessors = numValidSuccessors + 1;
                        sucessorNodes{numValidSuccessors,1} = neighborNode;
                        trajectoriesToSuccessors{numValidSuccessors,1} = neighborLocalTrajectory;
                        errToSuccessor(numValidSuccessors,1) = thisErr;
                        PMatToSuccessor{numValidSuccessors,1} = thisPMat;
                    end
                end
                
                
                % now copy over the successors into the neighbor spots
                newNode.neighbors = sucessorNodes(1:numValidSuccessors,1);
                newNode.errToNeighbors = errToSuccessor(1:numValidSuccessors,1);
                newNode.PMatToNeighbors = PMatToSuccessor(1:numValidSuccessors,1);
                newNode.trajectoriesToNeighbors = trajectoriesToSuccessors(1:numValidSuccessors,1);
                
            end
            
            % now add the newly created node to the queue (or update its position if
            % if it is the goal node and already in there... update() handles
            % the case the node is already in the queue automatically)
            update(Q, newNode, min(newNode.err,newNode.lmc));
            
            % now we do the replan (i.e. the make consistant) step
            while(~isempty(topKey(Q)))  %% && topKey(Q) <= min(G.goalNode.err, G.goalNode.lmc) )
                % while the heap is nonempty and the top nodes is at a lower level set than the goal
                
                
                thisNode = pop(Q);                        % get top node
                thisNode.err = thisNode.lmc;    % set cost to lmc
                % disp(thisNode.id)
                % now we need to see if any of the nodes that can be accessed
                % from this node would like to use this node as their parent
                
                for n_i = 1:length(thisNode.neighbors)
                    successorNode = thisNode.neighbors{n_i};
                    if successorNode.id == thisNode.parentNode.id
                        continue  % explicitly avoid parent loops (in cases of numerical rounding error this could be an issue)
                    elseif successorNode.id == G.startNode.id
                        continue  % by definition the start node cannot be improved
                    end
                    
                    errFromStartViaNewNode = thisNode.err + thisNode.errToNeighbors(n_i,1);
                    if successorNode.lmc > errFromStartViaNewNode
                        successorNode.lmc = errFromStartViaNewNode;
                        successorNode.parentID = thisNode.id;
                        successorNode.parentNode = thisNode;
                        successorNode.parentTraj = thisNode.trajectoriesToNeighbors{n_i,1};
                        successorNode.PMat = thisNode.PMatToNeighbors{n_i,1};
                    end
                    
                    if successorNode.err == successorNode.lmc
                        remove(Q, successorNode);
                    else
                        update(Q, successorNode, min(successorNode.err,successorNode.lmc));
                    end
                end
                
                
                
                
            end
            
        end
        
        
        if algorithmToUse == 0
            disp(num2str(G.n))
        end
        
        
        
        % ------------------- plot stuff -----------------
        if drawEachStep
            if problemDefinition == 1
                fig = figure(1);
                fig.Units = 'inches';
                fig.Position = [0 0 6 4];
                subplot(1,2,1)
                %G.plotEdges('Color',[0.7,0.7,0.7])  % draws edges gray
                hold on
                G.calculateParentEdgesForPlotting()     % note that this takes a bit of time, only use for demo
                G.plotParentEdges('r', 'LineWidth', 2)
                axis equal
                hold on
                
                %G.plotNodesWithInListVal(0, '.k', 'MarkerSize', 3)  % unvisited
                %G.plotNodesWithInListVal(1, 'or', 'MarkerSize', 3)  % open
                %G.plotNodesWithInListVal(2, 'ok', 'MarkerSize', 3)  % closed
                
                G.plotStartNode3DPosition('ob', 'MarkerSize', 10, 'LineWidth', 3)
                G.plotGoalNode3DPosition( 'xb', 'MarkerSize', 10, 'LineWidth', 3)
                
                hold off
                axis(ax_bds(1:6))
                grid on
                
                xlabel('X')
                ylabel('Y')
                zlabel('Z')
                title('Position')
                
                subplot(1,2,2)
                G.plotParentRotEdges('r', 'LineWidth', 2)
                hold on
                
                G.plotStartNode3DOrientation('ob', 'MarkerSize', 10, 'LineWidth', 3)
                G.plotGoalNode3DOrientation('xb', 'MarkerSize', 10, 'LineWidth', 3)
                hold off
                axis(ax_bds(7:12))
                grid on
                xlabel('Xrot')
                ylabel('Yrot')
                zlabel('Zrot')
                title('Orientation')
                pause(.000)
            elseif problemDefinition == 2
                fig = figure(1);
                fig.Units = 'inches';
                fig.Position = [0 0 6 4];
                %         G.plotEdges('Color',[0.7,0.7,0.7])  % draws edges gray
                %         hold on
                G.calculateParentEdgesForPlotting()     % note that this takes a bit of time, only use for demo
                G.plotParentEdges('r', 'LineWidth', 2)
                
                axis(ax_bds)
                axis equal
                hold on
                
                %         G.plotNodesWithInListVal(0, '.k', 'MarkerSize', 3)  % unvisited
                %         G.plotNodesWithInListVal(1, 'or', 'MarkerSize', 3)  % open
                %         G.plotNodesWithInListVal(2, 'ok', 'MarkerSize', 3)  % closed
                
                G.plotStartNode2DPosition('ob', 'MarkerSize', 10, 'LineWidth', 3)
                G.plotGoalNode2DPosition( 'xb', 'MarkerSize', 10, 'LineWidth', 3)
                
                corners = [-141.421356237310, 639.736528676648;
                    0, 781.157884913957;
                    141.421356237310, 639.736528676648;
                    0, 498.315172439338;
                    -141.421356237310, 639.736528676648];
                plot(corners(:,1),corners(:,2),'--k','linewidth',2)   
                hold off
                axis equal
                axis(ax_bds)
                xlabel('X')
                ylabel('Y')
                % Make a video:
                F = getframe(fig);
                Xs(:,:,:,jj) = frame2im(F);
                jj = jj +1;
                pause(.000)
                
            end
        end
        % ------------------- end plot stuff -----------------
        pause(0.0)
        %}
        
        if goalInKDTree == true
            Loop = Loop +1;
            G.extractMotionPath();
            Data(TN).Path(Loop).IT = G.motionPath;
            Error = NaN(size(G.motionPath,2),1);
            j = 0;
            for i = size(G.motionPath,2):-1:1
                j = j+1;
                Error(j) = G.nodes{G.motionPath(1,i),1}.err;
            end
            Data(TN).Err(Loop).IT = Error;
            Data(TN).Time(Loop,1) = toc(tstart);
            Data(TN).NumNodes(Loop,1) = G.n;
        end
        tcur=toc(tstart);
        
        
        
    end
    




    %
    % Extract Path
    G.extractMotionPath()
    
    calculatePlottingData(G)
    trajForPlotting = getParentTrajectoriesForPlotting(G,numDims);
    colorTrajForPlotting = getColorParentTrajectoriesForPlotting(G,numDims+1);
    bestPathTrajForPlotting = getBestPathTrajectoriesForPlotting(G,numDims);
    
    
    %set(0,'DefaultFigureVisible','on');
%{
    %% Make Video:
    v = VideoWriter('RRTsharp','MPEG-4');
    v.FrameRate = 50;
    open(v)
    for i = 1:size(Xs,4)
        writeVideo(v,Xs(:,:,:,i));
    end
    close(v)
%}

  % ------------------- plot stuff -----------------
    
    if EFV == 1
        EFVName = 'Ellipse';
    elseif EFV == 2
        EFVName = 'Trace';
    end
    %{
    if problemDefinition == 1
        if Count == 1
            fileloc = [pwd,'\Plots\SP\',date,'\RRTstarRVal1\',EFVName,'\',num2str(0.1),'\'];
            if ~isfolder(fileloc)
                mkdir(fileloc)
            end
        elseif Count == 2
            fileloc = [pwd,'\Plots\SP\',date,'\Replan1\',EFVName,'\',num2str(1),'\'];
            if ~isfolder(fileloc)
                mkdir(fileloc)
            end
        elseif Count == 3
            fileloc = [pwd,'\Plots\SP\',date,'\RRTstarRVal1\',EFVName,'\',num2str(10),'\'];
            if ~isfolder(fileloc)
                mkdir(fileloc)
            end
        end
    elseif problemDefinition == 2
        if Count == 1
            fileloc = [pwd,'\Plots\2D\',date,'\RRTstarRVal1\',EFVName,'\',num2str(0.1),'\'];
            if ~isfolder(fileloc)
                mkdir(fileloc)
            end
        elseif Count == 2
            fileloc = [pwd,'\Plots\2D\',date,'\RRTstarRVal1\',EFVName,'\',num2str(1),'\'];
            if ~isfolder(fileloc)
                mkdir(fileloc)
            end
        elseif Count == 3
            fileloc = [pwd,'\Plots\2D\',date,'\RRTstarRVal1\',EFVName,'\',num2str(10),'\'];
            if ~isfolder(fileloc)
                mkdir(fileloc)
            end
        end
    end
    %}
    
    fileloc = [pwd,'\Plots\SP\',date,'\NewPositions2\',EFVName,'\',num2str(Count),'\'];
    if ~isfolder(fileloc)
        mkdir(fileloc)
    end
    fig = figure(2);
    fig.Units = 'inches';
    fig.Position = [0 0 6 4];
    if numDims > 2
        subplot(1,2,1)
        G.plotEdges('Color',[0.7,0.7,0.7])  % draws edges gray
        hold on
        G.calculateParentEdgesForPlotting()
        
        G.plotParentEdges('r','LineWidth', 2)
        
        G.plotNodesWithInListVal(0, '.k', 'MarkerSize', 3)  % unvisited
        G.plotNodesWithInListVal(1, 'or', 'MarkerSize', 3)  % open
        G.plotNodesWithInListVal(2, 'ok', 'MarkerSize', 3)  % closed
        
        G.plotMotionPath('b',  'LineWidth', 4)
        
        G.plotStartNode('ob', 'MarkerSize', 10, 'LineWidth', 3)
        G.plotGoalNode( 'xb', 'MarkerSize', 10, 'LineWidth', 3)
        grid on
        hold off
        axis(ax_bds(1:6))
        axis equal
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        title('Position')
        
        subplot(1,2,2)
        G.plotRotEdges('Color',[0.7,0.7,0.7])  % draws edges gray
        hold on
        G.calculateParentEdgesForPlotting()
        
        G.plotParentRotEdges('r','LineWidth', 2)
        
        
        G.plotNodesWithInListValRot(0, '.k', 'MarkerSize', 3)  % unvisited
        G.plotNodesWithInListValRot(1, 'or', 'MarkerSize', 3)  % open
        G.plotNodesWithInListValRot(2, 'ok', 'MarkerSize', 3)  % closed
        
        G.plotRotMotionPath('b',  'LineWidth', 4)
        
        G.plotStartNodeRot('ob', 'MarkerSize', 10, 'LineWidth', 3)
        G.plotGoalNodeRot( 'xb', 'MarkerSize', 10, 'LineWidth', 3)
        grid on
        hold off
        axis(ax_bds(7:12))
        axis equal
        xlabel('Xrot')
        ylabel('Yrot')
        zlabel('Zrot')
        title('Orientation')
    else
        G.plotEdges('Color',[0.7,0.7,0.7])  % draws edges gray
        hold on
        G.calculateParentEdgesForPlotting()
        
        G.plotParentEdges('r','LineWidth', 2)
        
        G.plotNodesWithInListVal(0, '.k', 'MarkerSize', 3)  % unvisited
        G.plotNodesWithInListVal(1, 'or', 'MarkerSize', 3)  % open
        G.plotNodesWithInListVal(2, 'ok', 'MarkerSize', 3)  % closed
        
        %G.plotMotionPath('b',  'LineWidth', 4)
        
        G.plotStartNode('ob', 'MarkerSize', 10, 'LineWidth', 3)
        G.plotGoalNode( 'xb', 'MarkerSize', 10, 'LineWidth', 3)
        grid on
        hold off
        axis equal
        axis(ax_bds(1:4))
        
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        title('Position')
    end
    
    % now plot just the search tree
    fig = figure(3);
    fig.Units = 'inches';
    fig.Position = [0 0 6 4];
    if numDims > 2
        subplot(1,2,1)
        G.plotParentEdges('r', 'LineWidth', 2)
        axis(ax_bds(1:6))
        hold on
        
        G.plotMotionPath('b',  'LineWidth', 4)
        
        G.plotStartNode('ob', 'MarkerSize', 10, 'LineWidth', 3)
        G.plotGoalNode( 'xb', 'MarkerSize', 10, 'LineWidth', 3)
        grid on
        hold off
        axis equal
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        title('Position')
        
        subplot(1,2,2)
        G.plotParentRotEdges('r', 'LineWidth', 2)
        axis(ax_bds(7:12))
        axis equal
        hold on
        
        G.plotRotMotionPath('b',  'LineWidth', 4)
        
        G.plotStartNodeRot('ob', 'MarkerSize', 10, 'LineWidth', 3)
        G.plotGoalNodeRot( 'xb', 'MarkerSize', 10, 'LineWidth', 3)
        grid on
        hold off
        axis equal
        xlabel('Xrot')
        ylabel('Yrot')
        zlabel('Zrot')
        title('Orientation')
    else
        G.plotParentEdges('r', 'LineWidth', 2)
        
        hold on
        
        G.plotMotionPath('b',  'LineWidth', 4)
        
        G.plotStartNode('ob', 'MarkerSize', 10, 'LineWidth', 3)
        G.plotGoalNode( 'xb', 'MarkerSize', 10, 'LineWidth', 3)
        grid on
        corners = [-141.421356237310, 639.736528676648;
            0, 781.157884913957;
            141.421356237310, 639.736528676648;
            0, 498.315172439338;
            -141.421356237310, 639.736528676648];
        plot(corners(:,1),corners(:,2),'--k','linewidth',2)
        hold off
        axis equal
        axis(ax_bds(1:4))
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        title('Position')
    end
    
    % Save figure to file:
    if problemDefinition  == 1
        filename = [fileloc,'SPTN',num2str(TN),'FullTree.png'];
    else
        filename = [fileloc,'2DTN',num2str(TN),'FullTree.png'];
    end
    saveas(fig,filename);
    %}
    fig = figure(4);
   fig.Units = 'inches';
    fig.Position = [0 0 6 4];
    if numDims > 2
        subplot(1,2,1)
        plot3(trajForPlotting(:,1), trajForPlotting(:,2), trajForPlotting(:,3),'m', 'LineWidth', 2)
        axis(ax_bds(1:6))
        axis equal
        hold on
        plot3(bestPathTrajForPlotting(:,1), bestPathTrajForPlotting(:,2), bestPathTrajForPlotting(:,3), 'b', 'LineWidth', 2)
        G.plotStartNode('ok', 'MarkerSize', 10, 'LineWidth', 1)
        G.plotGoalNode( 'xk', 'MarkerSize', 10, 'LineWidth', 1)
        grid on
        
        for i = size(G.motionPath,2):-1:1
            [V,D] = eig(G.nodes{G.motionPath(1,i),1}.PMat);
            C = G.nodes{G.motionPath(1,i),1}.position;
            A = V*D;
            Axes = sqrt(diag(D));
            PA = Axes;
            % PA = PA + Axes;
            [x,y,z] = ellipsoid(C(1),C(2),C(3),PA(1),PA(2),PA(3));
            Ell = surf(x,y,z);
            Ell.FaceAlpha = 0;
            Ell.EdgeColor = 'cyan';
            if i ~=size(G.motionPath,2)
                theta = acos((trace(A)-1)/2);
                K = (1/(2*sin(theta)))*[A(3,2)-A(2,3);A(1,3)-A(3,1);A(2,1)-A(1,2)];
                rotate(Ell,K',rad2deg(theta),[C(1),C(2),C(3)]);
            end
        end
        hold off;
        axis equal
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        title('Position')
        
        subplot(1,2,2)
        plot3(rad2deg(trajForPlotting(:,4)), rad2deg(trajForPlotting(:,5)), rad2deg(trajForPlotting(:,6)),'m', 'LineWidth', 2)
        axis(ax_bds(7:12))
        axis equal
        hold on
        plot3(rad2deg(bestPathTrajForPlotting(:,4)), rad2deg(bestPathTrajForPlotting(:,5)), rad2deg(bestPathTrajForPlotting(:,6)), 'b', 'LineWidth', 2)
        G.plotStartNodeRot('ok', 'MarkerSize', 10, 'LineWidth', 1)
        G.plotGoalNodeRot( 'xk', 'MarkerSize', 10, 'LineWidth', 1)
        grid on
        
        for i = size(G.motionPath,2):-1:1
            [V,D] = eig(G.nodes{G.motionPath(1,i),1}.PMat);
            C = G.nodes{G.motionPath(1,i),1}.position;
            Axes = sqrt(diag(D));
            C(4:6) = rad2deg(C(4:6));
            PA = Axes;
            % PA = PA + Axes;
            A = V*D;
            [x,y,z] = ellipsoid(C(4),C(5),C(6),PA(4),PA(5),PA(6));
            Ell = surf(x,y,z);
            Ell.FaceAlpha = 0;
            Ell.EdgeColor = 'cyan';
            if i ~=size(G.motionPath,2)
                theta = acos((trace(A)-1)/2);
                K = (1/(2*sin(theta)))*[A(6,5)-A(5,6);A(4,6)-A(6,4);A(5,4)-A(4,5)];
                rotate(Ell,K',rad2deg(theta),[C(4),C(5),C(6)]);
            end
        end
        hold off;
        xlabel('Xrot')
        ylabel('Yrot')
        zlabel('Zrot')
        title('Orientation')
    else
        plot(trajForPlotting(:,1), trajForPlotting(:,2),'m', 'LineWidth', 2)
        axis(ax_bds(1:4))
        axis equal
        hold on
        plot(bestPathTrajForPlotting(:,1), bestPathTrajForPlotting(:,2), 'b', 'LineWidth', 2)
        G.plotStartNode('ok', 'MarkerSize', 10, 'LineWidth', 1)
        G.plotGoalNode( 'xk', 'MarkerSize', 10, 'LineWidth', 1)
        grid on
        
        for i = size(G.motionPath,2):-1:1
            [V,D] =  eig(G.nodes{G.motionPath(1,i),1}.PMat);
            C = G.nodes{G.motionPath(1,i),1}.position;
            A = V*D;
            PA = sqrt(diag(D));
            % PA = PA + sqrt(diag(D));
            t = -pi:0.01:pi;
            x = C(1) + PA(1)*cos(t);
            y = C(2) + PA(2)*sin(t);
            Ell = plot(x,y,'--c');
            if i ~= size(G.motionPath,2)
                theta = atan2(A(2,1),A(1,1));
                rotate(Ell,[0 0 1],theta,[C(1) C(2) 0]);
            end
        end
        corners = [-141.421356237310, 639.736528676648;
            0, 781.157884913957;
            141.421356237310, 639.736528676648;
            0, 498.315172439338;
            -141.421356237310, 639.736528676648];
        plot(corners(:,1),corners(:,2),'--k','linewidth',2)
        hold off;
        axis equal
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        title('Position')
        
    end
    
    % Save figure to file:
    if problemDefinition == 1
        filename = [fileloc,'SPTN',num2str(TN),'AllTrajs.png'];
    else
        filename = [fileloc,'2DTN',num2str(TN),'AllTrajs.png'];
    end
    saveas(fig,filename);
    
    fig = figure(5);
    fig.Units = 'inches';
    fig.Position = [0 0 6 4];
    if numDims > 2
        subplot(1,2,1)
        %plot3(trajForPlotting(:,1), trajForPlotting(:,2), trajForPlotting(:,3),'m', 'LineWidth', 2)
        axis(ax_bds(1:6))
        axis equal
        hold on
        plot3(bestPathTrajForPlotting(:,1), bestPathTrajForPlotting(:,2), bestPathTrajForPlotting(:,3), 'b', 'LineWidth', 2)
        G.plotStartNode('ok', 'MarkerSize', 10, 'LineWidth', 1)
        G.plotGoalNode( 'xk', 'MarkerSize', 10, 'LineWidth', 1)
        grid on
        
        for i = size(G.motionPath,2):-1:1
            [V,D] = eig(G.nodes{G.motionPath(1,i),1}.PMat);
            C = G.nodes{G.motionPath(1,i),1}.position;
            A = V*D;
            Axes = sqrt(diag(D));
            PA = Axes;
            % PA = PA + Axes;
            [x,y,z] = ellipsoid(C(1),C(2),C(3),PA(1),PA(2),PA(3));
            Ell = surf(x,y,z);
            Ell.FaceAlpha = 0;
            Ell.EdgeColor = 'cyan';
            if i ~=size(G.motionPath,2)
                theta = acos((trace(A)-1)/2);
                K = (1/(2*sin(theta)))*[A(3,2)-A(2,3);A(1,3)-A(3,1);A(2,1)-A(1,2)];
                rotate(Ell,K',rad2deg(theta),[C(1),C(2),C(3)]);
            end
        end
        hold off;
        axis equal
        axis(ax_bds(1:6))
        
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        title('Position')
        
        subplot(1,2,2)
        %plot3(rad2deg(trajForPlotting(:,4)), rad2deg(trajForPlotting(:,5)), rad2deg(trajForPlotting(:,6)),'m', 'LineWidth', 2)
        axis(ax_bds(7:12))
        axis equal
        hold on
        plot3(rad2deg(bestPathTrajForPlotting(:,4)), rad2deg(bestPathTrajForPlotting(:,5)), rad2deg(bestPathTrajForPlotting(:,6)), 'b', 'LineWidth', 2)
        G.plotStartNodeRot('ok', 'MarkerSize', 10, 'LineWidth', 1)
        G.plotGoalNodeRot( 'xk', 'MarkerSize', 10, 'LineWidth', 1)
        grid on
        PA = zeros(6,1);
        for i = size(G.motionPath,2):-1:1
            [V,D] = eig(G.nodes{G.motionPath(1,i),1}.PMat);
            C = G.nodes{G.motionPath(1,i),1}.position;
            C(4:6) = rad2deg(C(4:6));
            Axes = sqrt(diag(D));
            PA = Axes;
            % PA = PA + Axes;
            A = V*D;
            [x,y,z] = ellipsoid(C(4),C(5),C(6),PA(4),PA(5),PA(6));
            Ell = surf(x,y,z);
            Ell.FaceAlpha = 0;
            Ell.EdgeColor = 'cyan';
            if i ~=size(G.motionPath,2)
                theta = acos((trace(A)-1)/2);
                K = (1/(2*sin(theta)))*[A(6,5)-A(5,6);A(4,6)-A(6,4);A(5,4)-A(4,5)];
                rotate(Ell,K',rad2deg(theta),[C(4),C(5),C(6)]);
            end
        end
        axis(ax_bds(7:12))
        hold off;
        xlabel('Xrot')
        ylabel('Yrot')
        zlabel('Zrot')
        title('Orientation')
    else
        %plot(trajForPlotting(:,1), trajForPlotting(:,2),'m', 'LineWidth', 2)
        axis(ax_bds(1:4))
        axis equal
        hold on
        plot(bestPathTrajForPlotting(:,1), bestPathTrajForPlotting(:,2), 'b', 'LineWidth', 2)
        G.plotStartNode('ok', 'MarkerSize', 10, 'LineWidth', 1)
        G.plotGoalNode( 'xk', 'MarkerSize', 10, 'LineWidth', 1)
        grid on
        PA = zeros(2,1);
        for i = size(G.motionPath,2):-1:1
            [V,D] =  eig(G.nodes{G.motionPath(1,i),1}.PMat);
            C = G.nodes{G.motionPath(1,i),1}.position;
            A = V*D;
            PA = sqrt(diag(D));
            % PA = PA + sqrt(diag(D));
            t = -pi:0.01:pi;
            x = C(1) + PA(1)*cos(t);
            y = C(2) + PA(2)*sin(t);
            Ell = plot(x,y,'--c');
            if i ~= size(G.motionPath,2)
                theta = atan2(A(2,1),A(1,1));
                rotate(Ell,[0 0 1],theta,[C(1) C(2) 0]);
            end
        end
        corners = [-141.421356237310, 639.736528676648;
            0, 781.157884913957;
            141.421356237310, 639.736528676648;
            0, 498.315172439338;
            -141.421356237310, 639.736528676648];
        plot(corners(:,1),corners(:,2),'--k','linewidth',2)
        
        % Plot nodes that have infinite cost:
        if algorithmToUse == 2
            for i = 2:G.n
                if G.nodes{i,1}.err == inf
                    plot(G.nodes{i,1}.position(1),G.nodes{i,1}.position(1),'xg','linewidth',2);
                    disp('inf err')
                elseif G.nodes{i,1}.err ~= G.nodes{i,1}.lmc
                    disp('err ~= lmc')
                    if G.nodes{i,1}.err < G.nodes{i,1}.lmc
                        plot(G.nodes{i,1}.position(1),G.nodes{i,1}.position(1),'xr','linewidth',2);
                    elseif G.nodes{i,1}.err > G.nodes{i,1}.lmc
                        plot(G.nodes{i,1}.position(1),G.nodes{i,1}.position(1),'xy','linewidth',2);
                    end
                end
            end
        end
        hold off;
        axis equal
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        title('Position')
    end
    % Save figure to file:
    if problemDefinition  == 1
        filename = [fileloc,'SPTN',num2str(TN),'ChosenTraj.png'];
    else
        filename = [fileloc,'2DTN',num2str(TN),'ChosenTraj.png'];
    end
    saveas(fig,filename);
    %}
    
    % Plot Cost from Start along the Path:
    fig = figure(6);
    fig.Units = 'inches';
    fig.Position = [0 0 3 2];
    if numDims > 2
        j = 0;
        for i = size(G.motionPath,2):-1:1
            j=j+1;
            xAx(j) = j;
            yAxP(j) = G.nodes{G.motionPath(i)}.err;
        end
        plot(xAx,yAxP);
        xlim([0, max(xAx)]);
        xlabel('Node on Path');
        ylabel('Position Error');
        title('Error Growth along Path');
    else
        j = 0;
        for i = size(G.motionPath,2):-1:1
            j=j+1;
            xAx(j) = j;
            yAx1(j) = G.nodes{G.motionPath(i)}.err;
        end
        plot(xAx,yAx1);
        xlim([0, max(xAx)]);
        xlabel('Node on Path');
        ylabel('Error');
        title('Error Growth along Path');
        
    end
    
    % Save figure to file:
    if problemDefinition  == 1
        filename = [fileloc,'SPTN',num2str(TN),'ErrorGrowth.png'];
    else
        filename = [fileloc,'2DTN',num2str(TN),'ErrorGrowth.png'];
    end
    saveas(fig,filename);
    
    %}
    % Plot based on color:
    ColorCode(G,colorTrajForPlotting,bestPathTrajForPlotting);
    % Save figure to file:
    if problemDefinition == 1
        filename = [fileloc,'SPTN',num2str(TN),'ColorCode.png'];
    else
        filename = [fileloc,'2DTN',num2str(TN),'ColorCode.png'];
    end
    saveas(gcf,filename);
    %}
    
    SaveParaData2(G,TN,QMat,problemDefinition,fileloc)
    SaveFullData2(G,TN,problemDefinition,fileloc)
end


set(0,'DefaultFigureVisible','on');

map = jet(size(Data,2));
fig = figure(7);
fig.Units = 'inches';
fig.Position = [0 0 3 2];
for j = 1:TN
    Err(j).End = NaN(size(Data(j).Err,2),1);
    for i = 1:size(Data(j).Err,2)
        Err(j).End(i) = Data(j).Err(i).IT(end);
        Err(j).Time(i,1) = Data(j).Time(i);
    end
    BestErr(j) = Err(j).End(end);
    ErrPlot = plot(Err(j).Time,Err(j).End);
    ErrPlot.Color = map(j,:);
    ErrPlot.LineWidth = 2;
    ErrPlot.DisplayName = strcat('Trial #',num2str(j));
    hold on
end
legend
hold off
xlabel('Time [s]')
ylabel('Error')
title('Error Reduction After Iterating')
if problemDefinition == 1
    filename = [fileloc,'SP','TrialTrack.png'];
else
    filename = [fileloc,'2D','TrialTrack.png'];
end
saveas(gcf,filename);

fig = figure(8);
fig.Units = 'inches';
fig.Position = [0 0 3 2];
plot(BestErr)
xlabel('Trial Number')
ylabel('Ending Error')
title('Ending Error of Each Trial')

fig = figure(9);
fig.Units = 'inches';
fig.Position = [0 0 3 2];
histfit(BestErr)
xlabel('Ending Error')
ylabel('# of Trials')
title('Ending Errors')
if problemDefinition == 1
    filename = [fileloc,'SP','HistFit.png'];
else
    filename = [fileloc,'2D','HistFit.png'];
end
saveas(gcf,filename);
fig = figure(10);
fig.Units = 'inches';
fig.Position = [0 0 3 2];
hold on
for i = 1:TN
    plot(Data(i).Time,Data(i).NumNodes,'blue');
end
xlabel('Time [s]');
ylabel('Number of Nodes');
title('Tree Expansion over Time');
grid on
hold off
if problemDefinition == 1
    filename = [fileloc,'SP','GrowthRate.png'];
else
    filename = [fileloc,'2D','GrowthRate.png'];
end
saveas(gcf,filename);


AverageValuesData(Data,tend,problemDefinition);
if problemDefinition == 1
    filename = [fileloc,'SP','BoxPlots.png'];
else
    filename = [fileloc,'2D','BoxPlots.png'];
end
saveas(gcf,filename);


FullDataDump(Data,fileloc);
end
end
%}